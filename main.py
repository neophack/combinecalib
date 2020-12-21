import random
import string

import cherrypy
import os
import json
from jinja2 import Environment, FileSystemLoader
env = Environment(loader=FileSystemLoader('./'))

import os
import sys
import scene_reader
import math

import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

sys.path.append(os.path.join(BASE_DIR, './algos'))
import algos.rotation as rotation

#sys.path.append(os.path.join(BASE_DIR, '../tracking'))
import algos.trajectory as trajectory

extract_object_exe = "~/code/pcltest/build/extract_object"
registration_exe = "~/code/go_icp_pcl/build/test_go_icp"


def euler_angle_to_rotate_matrix(eu,tr):
    theta=[eu["x"],eu["y"],eu["z"]]
    # print(theta)
    # theta=[1,1,1]
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])
 
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])
 
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_x, np.dot(R_y, R_z))
    return np.array([
        [R[0,0],R[0,1],R[0,2], tr["x"]],
        [R[1,0],R[1,1],R[1,2], tr["y"]],
        [R[2,0],R[2,1],R[2,2], tr["z"]],
        [0,          0,          0,          1],
    ])


def psr_to_xyz(p,s,r):
    trans_matrix=euler_angle_to_rotate_matrix(r,p)
    print(trans_matrix)
    x=s["x"]/2
    y=s["y"]/2
    z=s["z"]/2
    print("xzy:",x,y,z)
    # local_coord=np.array([
    #     [x, y, -z, 1],   [x, -y, -z, 1],  #front-left-bottom, front-right-bottom
    #     [x, -y, z, 1],   [x, y, z, 1],  #front-right-top,   front-left-top

    #     [-x, y, -z, 1],   [-x, -y, -z, 1],  #rear-left-bottom, rear-right-bottom
    #     [-x, -y, z, 1],   [-x, y, z, 1],  #rear-right-top,   rear-left-top
    # ])
    # local_coord=np.array([
    #     [x, y, -z, 1],   [x, -y, -z, 1],  #front-left-bottom, front-right-bottom
    #     [x, -y, z, 1],   [x, y, z, 1],  #front-right-top,   front-left-top

    #     [-x, y, -z, 1],   [-x, -y, -z, 1],  #rear-left-bottom, rear-right-bottom
    #     [-x, -y, z, 1],   [-x, y, z, 1],  #rear-right-top,   rear-left-top
    # ])
    local_coord=np.array([
        
        [-x, y, z, 1],[-x, -y, z, 1],[-x, -y, -z, 1],[-x, y, -z, 1]

    ])
    print(local_coord.T)
    world_coord = np.dot(trans_matrix, local_coord.T)
    print(world_coord)
    w = world_coord
    return w

def psr_to_xyzcenter(p,s,r):
    trans_matrix=euler_angle_to_rotate_matrix(r,p)
    print(trans_matrix)
    x=s["x"]/2
    y=s["y"]/2
    z=s["z"]/2

    local_coord=np.array([
        [0, 0, 0, 1], #left-top
    ])
    world_coord = np.dot(trans_matrix, local_coord.T)
    w = world_coord
    return w


# 欧拉角转换为四元数, 旋转顺序为ZYX(偏航角yaw, 俯仰角pitch, 横滚角roll)
def eular2quat(yaw, pitch, roll):
    # 注意这里必须先转换为弧度, 因为这里的三角计算均使用的是弧度.
    # yaw = math.radians(yaw)
    # pitch = math.radians(pitch)
    # roll = math.radians(roll)
    roll=-roll
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
 
    # 笛卡尔坐标系
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
 
    # Direct3D, 笛卡尔坐标的X轴变为Z轴, Y轴变为X轴, Z轴变为Y轴
    # w = cr * cp * cy + sr * sp * sy
    # x = cr * sp * cy + sr * cp * sy
    # y = cr * cp * sy - sr * sp * cy
    # z = sr * cp * cy - cr * sp * sy
 
    return w, x, y, z


class Root(object):
    @cherrypy.expose
    def index(self):
      tmpl = env.get_template('index.html')
      return tmpl.render()
  
    @cherrypy.expose
    def ml(self):
      tmpl = env.get_template('test_ml.html')
      return tmpl.render()
  
    @cherrypy.expose
    def reg(self):
      tmpl = env.get_template('registration_demo.html')
      return tmpl.render()

    @cherrypy.expose
    def view(self, file):
      tmpl = env.get_template('view.html')
      return tmpl.render()
          
    @cherrypy.expose
    def save(self, scene, frame):

      # cl = cherrypy.request.headers['Content-Length']
      rawbody = cherrypy.request.body.readline().decode('UTF-8')

      with open("./data/"+scene +"/label/"+frame+".json",'w') as f:
        f.write(rawbody)
      
      return "ok"
    @cherrypy.expose
    @cherrypy.tools.json_out()
    def interpolate(self, scene, frame, obj_id):
      interpolate_num = trajectory.predict(scene, obj_id, frame, None)
      return interpolate_num


    # data  N*3 numpy array
    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def predict_rotation(self):
      cl = cherrypy.request.headers['Content-Length']
      rawbody = cherrypy.request.body.read(int(cl))
      
      data = json.loads(rawbody)
      
      return {"angle": rotation.predict(data["points"])}
      #return {}

    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def load_annotation(self, scene, frame):
      return scene_reader.read_annotations(scene, frame)
    
    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def run_calibration(self,scene,cam,extrinsic):
      extlist=[float(x) for x in extrinsic.split(",")]
      #xi,yi,zi=(extlist[2],extlist[0],-extlist[1])
      xi,yi,zi=(extlist[0],extlist[1],extlist[2])
      print(xi,yi,zi)
      # extrinsicm=np.array().reshape(4,4)

      calibfile="./data/"+scene+"/calib/"+cam+".json"
      calib=json.load(open(calibfile))
      mtx=np.array(calib["intrinsic"]).reshape(3,3)

      labeldir="./data/"+scene +"/label/"
      labels=os.listdir(labeldir)
      calibdata=[]
      for labelfile in labels:
          print(labelfile)
          annos=json.load(open(os.path.join(labeldir,labelfile)))
          flagpsr=False
          flagpoly=False
          for anno in annos:
              if "psr" in anno :
                  
                  if anno["obj_type"]=="Plane":
                      rotation=anno["psr"]["rotation"]
                      position=anno["psr"]["position"]
                      scale=anno["psr"]["scale"]
                      flagpsr=True
              if "poly" in anno:
                if anno["poly"]["cam"]==cam:
                    poly=anno["poly"]["points"][:8]
                    flagpoly=True
              if flagpsr and flagpoly:
                  print("psr_to_xyz:",psr_to_xyz(position,scale,rotation))
                  calibdata.append((psr_to_xyz(position,scale,rotation),psr_to_xyzcenter(position,scale,rotation),poly))

      lidar=[]
      lidarcenter=[]
      imgc=[]
      imgcenter=[]
      for calib in calibdata:
          xyz=calib[0]
          xyzcenter=calib[1]
          if len(lidar):
              lidar=np.hstack((lidar,xyz))
              lidarcenter=np.hstack((lidarcenter,xyzcenter))
          else:
              lidar=xyz
              lidarcenter=xyzcenter
         
          poly=np.array(calib[2]).reshape(-1,2)
          # poly=poly[::2,:]
         
          polycenter=np.mean(poly, axis=0).reshape(-1,2)
          print(poly,polycenter)
          if len(imgc):
              imgc=np.vstack((imgc,poly))
              imgcenter=np.vstack((imgcenter,polycenter))
          else:
              imgc=poly
              imgcenter=polycenter
   
      # x_data= np.transpose(lidarcenter)
      # y_data=imgcenter

      x_data= np.transpose(lidar)
      # imgc=np.vstack((imgc,imgc))
      # imgc=imgc[::2,]
      y_data=imgc
      print(x_data,y_data)
      #print("shape:",x_data.shape,y_data.shape)

      xx=tf.placeholder(tf.float32,[None,4])
      yy=tf.placeholder(tf.float32,[None,2])

      # trmetric = tf.Variable(extrinsicm,dtype=tf.float32)
      tr=tf.Variable([xi,yi,zi,extlist[3],extlist[4],extlist[5]])
      # w=tr[0]
      # x=tr[1]
      # y=tr[2]
      # z=tr[3]

      # roll=-roll
      yaw=tr[0]
      pitch=tr[1]
      roll=tr[2]


      R_x = [
          [1,       0,              0],
          [0,       tf.cos(yaw),   -tf.sin(yaw)],
          [0,       tf.sin(yaw),   tf.cos(yaw)]
      ]

      #Calculate rotation about y axis
      R_y = [
          [tf.cos(pitch),      0,      tf.sin(pitch)],
          [0,                       1,      0],
          [-tf.sin(pitch),     0,      tf.cos(pitch)]
      ]

      #Calculate rotation about z axis
      R_z = [
          [tf.cos(roll),    -tf.sin(roll),      0],
          [tf.sin(roll),    tf.cos(roll),       0],
          [0,               0,                  1]
      ]

      #Combined rotation matrix

      R = tf.matmul(R_x, tf.matmul(R_y, R_z))
    
      trmetric = tf.stack([[R[0,0], R[0,1], R[0,2],tr[3]],
                          [R[1,0], R[1,1], R[1,2],tr[4]],
                          [R[2,0], R[2,1], R[2,2],tr[5]],
                          [0.,0.,0.,1.]])

    
      print(trmetric.shape)
      # trmetric = tf.Variable(tf.stack([[0.12050900002161516, 0.9764353500118469, -0.17902957342582415, 0.20487898588180542],
      #                                         [0.3029412509100283, -0.20791354640727297, -0.9300529854354632, 0.0013696063542738557],
      #                                         [-0.945359285885984, 0.057844312318437535, -0.3208579999338484, -0.10943480581045151],
      #                                         [0, 0, 0, 1]]))
   
      pr=tf.matmul(xx,tf.transpose(trmetric))[:,:3]
      # print(K.shape,pr.shape)
      K=tf.constant(mtx,dtype=tf.float32)
      # K= tf.Variable(mtx,dtype=tf.float32)
      pt=tf.matmul(pr,tf.transpose(K))
      print(pt.shape)
      # predict=pt[:,:2]
      predict=pt[:,:2]/tf.stack([pt[:,2],pt[:,2]],axis=1)
      print(predict.shape,yy.shape)
      #损失函数选用SME
      # loss=tf.reduce_mean(tf.square(tf.multiply(yy,tf.stack([pt[:,2],pt[:,2]],axis=1))/2048-predict/2048))
      diff=yy-predict
      
      loss=tf.reduce_mean(tf.square((diff)/100))
      #优化函数选取梯度下降法
      train=tf.train.AdamOptimizer(0.00001).minimize(loss)
      # train=tf.train.GradientDescentOptimizer(0.0001).minimize(loss)
      
      with tf.Session() as sess:
          predict_y = None
          sess.run(tf.global_variables_initializer())
          for i in range(5000):
              sess.run(train,feed_dict={xx:x_data,yy:y_data})
              # print(sess.run(trmetric))
              # print("R",sess.run(R,feed_dict={xx:x_data,yy:y_data}))
              # print("pred",np.hstack((sess.run(predict,feed_dict={xx:x_data,yy:y_data}),sess.run(predict,feed_dict={xx:x_data2,yy:y_data}))))
              #print("diff",sess.run(tf.stack([yy,predict],axis=1),feed_dict={xx:x_data,yy:y_data}))
              # print("K",sess.run(K,feed_dict={xx:x_data,yy:y_data}))
              # print(sess.run(tf.unstack((tr[:4]/tf.sqrt(tf.compat.v1.reduce_sum(tf.square(tr[:4]), axis=0))), num=4, axis=-1),feed_dict={xx:x_data,yy:y_data}))
              extrinsicm=sess.run(trmetric,feed_dict={xx:x_data,yy:y_data})
          print("diff",sess.run(tf.stack([yy,predict],axis=1),feed_dict={xx:x_data,yy:y_data}))
          print(extrinsicm)
      
      return {"extrinsic":extrinsicm.reshape(1,-1)[0].tolist()}

    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def auto_adjust(self, scene, ref_frame, object_id, adj_frame):
      
      #os.chdir("./temp")
      os.system("rm ./temp/src.pcd ./temp/tgt.pcd ./temp/out.pcd ./temp/trans.json")


      tgt_pcd_file = "./data/"+scene +"/pcd/"+ref_frame+".pcd"
      tgt_json_file = "./data/"+scene +"/label/"+ref_frame+".json"

      src_pcd_file = "./data/"+scene +"/pcd/"+adj_frame+".pcd"      
      src_json_file = "./data/"+scene +"/label/"+adj_frame+".json"

      cmd = extract_object_exe +" "+ src_pcd_file + " " + src_json_file + " " + object_id + " " +"./temp/src.pcd"
      print(cmd)
      os.system(cmd)

      cmd = extract_object_exe + " "+ tgt_pcd_file + " " + tgt_json_file + " " + object_id + " " +"./temp/tgt.pcd"
      print(cmd)
      os.system(cmd)

      cmd = registration_exe + " ./temp/tgt.pcd ./temp/src.pcd ./temp/out.pcd ./temp/trans.json"
      print(cmd)
      os.system(cmd)

      with open("./temp/trans.json", "r") as f:
        trans = json.load(f)
        print(trans)
        return trans

      return {}

    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def datameta(self):
      return scene_reader.get_all_scenes()
    
    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def objs_of_scene(self, scene):
      return self.get_all_unique_objs(os.path.join("./data",scene))

    def get_all_unique_objs(self, path):
      files = os.listdir(os.path.join(path, "label"))

      files = filter(lambda x: x.split(".")[-1]=="json", files)


      def file_2_objs(f):
          with open(f) as fd:
              boxes = json.load(fd)
              objs = [x for x in map(lambda b: {"category":b["obj_type"], "id": b["obj_id"]}, boxes)]
              return objs

      boxes = map(lambda f: file_2_objs(os.path.join(path, "label", f)), files)

      # the following map makes the category-id pairs unique in scene
      all_objs={}
      for x in boxes:
          for o in x:
              all_objs[o["category"]+"-"+o["id"]]=o

      objs = [x for x in all_objs.values()]
      #print(objs)
      #objs.sort()
      return objs

if __name__ == '__main__':
    cherrypy.quickstart(Root(), '/', config="server.conf")
else:
    application = cherrypy.Application(Root(), '/', config="server.conf")
