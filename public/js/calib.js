import { data } from "./data.js"
import { rotation_matrix_to_euler_angle, euler_angle_to_rotate_matrix } from "./util.js"
import { render_2d_image, update_image_box_projection } from "./image.js"
import { selected_box } from "./main.js"

var euler_angle = { x: 0, y: 0, z: 0 };
var translate = { x: 0, y: 0, z: 0 };
var params = {};

function save_calibration() {


    var scene_meta = data.meta.find(function (x) { return x.scene == data.world.file_info.scene; });


    var active_image_name = data.world.images.active_name;
    var calib = scene_meta.calib[active_image_name]

    var extrinsic = calib.extrinsic.map(function (x) { return x * 1.0; });

    euler_angle = rotation_matrix_to_euler_angle(extrinsic);
    translate = {
        x: extrinsic[3] * 1.0,
        y: extrinsic[7] * 1.0,
        z: extrinsic[11] * 1.0,
    };

    var extrinsicstr = JSON.stringify([extrinsic.slice(0, 4 * 1),extrinsic.slice(4 * 1, 4 * 2),extrinsic.slice(4 * 2, 4 * 3),extrinsic.slice(4 * 3, 4 * 4)]);
    console.log(extrinsicstr);
    console.log(euler_angle, translate);

    console.log("restoreed matrix", euler_angle_to_rotate_matrix(euler_angle, translate));

}

function reset_calibration() {
    var scene_meta = data.meta.find(function (x) { return x.scene == data.world.file_info.scene; });

    var active_image_name = data.world.images.active_name;
    var calib = scene_meta.calib[active_image_name]

    calib.extrinsic = euler_angle_to_rotate_matrix(euler_angle, translate);
    render_2d_image();

    if (selected_box)
        update_image_box_projection(selected_box);
}

function run_calibration() {
    console.log("calibration all!!!")
    var scene_meta = data.meta.find(function (x) { return x.scene == data.world.file_info.scene; });


    var active_image_name = data.world.images.active_name;
    var calib = scene_meta.calib[active_image_name]

    var extrinsic = calib.extrinsic.map(function (x) { return x * 1.0; });

    var xhr = new XMLHttpRequest();
    // we defined the xhr
    var _self = this;
    xhr.onreadystatechange = function () {
        if (this.readyState != 4) return;
    
        if (this.status == 200) {

            if (this.responseText.length == 0){
                
            }else {
                extrinsic=JSON.parse(this.responseText).extrinsic;
                console.log("extrinsic1",extrinsic);       
            }
            euler_angle = rotation_matrix_to_euler_angle(extrinsic);
            translate = {
                x: extrinsic[3] * 1.0,
                y: extrinsic[7] * 1.0,
                z: extrinsic[11] * 1.0,
            };
            params.sx = euler_angle.x;
            params.sy = euler_angle.y;
            params.sz = euler_angle.z;
            params.stx = translate.x;
            params.sty = translate.y;
            params.stz = translate.z;

        }
    
        // end of state change: it can be after some time (async)
        calib.extrinsic = extrinsic;//euler_angle_to_rotate_matrix(euler_angle, translate);
        render_2d_image();
    
        if (selected_box)
            update_image_box_projection(selected_box); 
    };
    euler_angle = rotation_matrix_to_euler_angle(calib.extrinsic);
    var extlist=[euler_angle.x,euler_angle.y,euler_angle.z,extrinsic[3],extrinsic[7],extrinsic[11]]
    xhr.open('GET', "/run_calibration"+"?scene="+data.world.file_info.scene+"&cam="+active_image_name+"&extrinsic="+extlist, true);
    xhr.send();

    // console.log("extrinsic2",extrinsic);   
    


   
}


function calibrate(ax, value) {
    var scene_meta = data.meta.find(function (x) { return x.scene == data.world.file_info.scene; });

    var active_image_name = data.world.images.active_name;
    var calib = scene_meta.calib[active_image_name]
    var extrinsic = calib.extrinsic.map(function (x) { return x * 1.0; });

    var euler_angle = rotation_matrix_to_euler_angle(extrinsic);
    var translate = {
        x: extrinsic[3] * 1.0,
        y: extrinsic[7] * 1.0,
        z: extrinsic[11] * 1.0,
    };
    // calibinfo={euler_angle:euler_angle,translate:translate};

    if (ax == 'z') {
        euler_angle.z += value;
    } else if (ax == 'x') {
        euler_angle.x += value;
    }
    else if (ax == 'y') {
        euler_angle.y += value;
    } else if (ax == 'tz') {
        translate.z += value;
    } else if (ax == 'tx') {
        translate.x += value;
    }
    else if (ax == 'ty') {
        translate.y += value;
    }
    params.sx = euler_angle.x;
    params.sy = euler_angle.y;
    params.sz = euler_angle.z;
    params.stx = translate.x;
    params.sty = translate.y;
    params.stz = translate.z;

    calib.extrinsic = euler_angle_to_rotate_matrix(euler_angle, translate);

    console.log("extrinsic", calib.extrinsic)
    console.log("euler", euler_angle, "translate", translate);

    render_2d_image();

    if (selected_box)
        update_image_box_projection(selected_box);
}

function updatecalibinfo() {
    var scene_meta = data.meta.find(function (x) { return x.scene == data.world.file_info.scene; });

    var active_image_name = data.world.images.active_name;
    if(active_image_name=="--camera--"){return;}
    var calib = scene_meta.calib[active_image_name]
    var extrinsic = calib.extrinsic.map(function (x) { return x * 1.0; });

    var euler_angle = rotation_matrix_to_euler_angle(extrinsic);
    var translate = {
        x: extrinsic[3] * 1.0,
        y: extrinsic[7] * 1.0,
        z: extrinsic[11] * 1.0,
    };
    params.sx = euler_angle.x;
    params.sy = euler_angle.y;
    params.sz = euler_angle.z;
    params.stx = translate.x;
    params.sty = translate.y;
    params.stz = translate.z;

}

function calibrateabs(ax, value) {
    var scene_meta = data.meta.find(function (x) { return x.scene == data.world.file_info.scene; });

    var active_image_name = data.world.images.active_name;
    var calib = scene_meta.calib[active_image_name]
    var extrinsic = calib.extrinsic.map(function (x) { return x * 1.0; });

    var euler_angle = rotation_matrix_to_euler_angle(extrinsic);
    var translate = {
        x: extrinsic[3] * 1.0,
        y: extrinsic[7] * 1.0,
        z: extrinsic[11] * 1.0,
    };


    if (ax == 'sz') {
        euler_angle.z = value;
    } else if (ax == 'sx') {
        euler_angle.x = value;
    }
    else if (ax == 'sy') {
        euler_angle.y = value;
    } else if (ax == 'stz') {
        translate.z = value;
    } else if (ax == 'stx') {
        translate.x = value;
    }
    else if (ax == 'sty') {
        translate.y = value;
    }

    params.sx = euler_angle.x;
    params.sy = euler_angle.y;
    params.sz = euler_angle.z;
    params.stx = translate.x;
    params.sty = translate.y;
    params.stz = translate.z;


    calib.extrinsic = euler_angle_to_rotate_matrix(euler_angle, translate);

    console.log("extrinsic", calib.extrinsic)
    console.log("euler", euler_angle, "translate", translate);

    render_2d_image();

    if (selected_box)
        update_image_box_projection(selected_box);
}



function install_calib_menu(parent_gui) {


    //calibrate
    var calibrateFolder = parent_gui.addFolder('Calibrate LiDAR-camera');
    params['save cal'] = function () {
        save_calibration();
    };
    calibrateFolder.add(params, 'save cal');

    params['reset cal'] = function () {
        reset_calibration();
    };
    calibrateFolder.add(params, 'reset cal');

    params['calibration'] = function () {
        run_calibration();
    };
    calibrateFolder.add(params, 'calibration');

    [
        { name: "sx", v: 0.0 },
        { name: "sy", v: 0.0 },
        { name: "sz", v: 0.0 }
    ].forEach(function (x) {
        var item_name = x.name;
        params[item_name] = x.v;
        var controler;

        if (item_name == "sx") {
            controler = calibrateFolder.add(params, item_name, 0, 3.14, 0.001);
        } else {
            controler = calibrateFolder.add(params, item_name, -3.14, 3.14, 0.001);
        }
        controler.onChange(function (value) {
            console.log("onChange:" + item_name + " " + value);
            calibrateabs(x.name, value);
        });
        controler.listen();
    });
    [
        { name: "stx", v: 0.0 },
        { name: "sty", v: 0.0 },
        { name: "stz", v: 0.0 }
    ].forEach(function (x) {
        var item_name = x.name;
        params[item_name] = x.v;
        var controler = calibrateFolder.add(params, item_name, -2.0, 2.0, 0.001);
        controler.onChange(function (value) {
            console.log("onChange:" + item_name + " " + value);
            calibrateabs(x.name, value);
        });
        controler.listen();
    });

    [
        { name: "x", v: 0.002 },
        { name: "x", v: -0.002 },
        { name: "y", v: 0.002 },
        { name: "y", v: -0.002 },
        { name: "z", v: 0.002 },
        { name: "z", v: -0.002 },

        { name: "tx", v: 0.005 },
        { name: "tx", v: -0.005 },
        { name: "ty", v: 0.005 },
        { name: "ty", v: -0.005 },
        { name: "tz", v: 0.005 },
        { name: "tz", v: -0.005 },
    ].forEach(function (x) {
        var item_name = x.name + "," + x.v;
        params[item_name] = function () {
            calibrate(x.name, x.v);
        };
        calibrateFolder.add(params, item_name);
    });

}
export { install_calib_menu, updatecalibinfo }