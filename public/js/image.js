
import { data } from "./data.js"
import { params, selected_box, select_bbox } from "./main.js"
import { vector4to3, vector3_nomalize, psr_to_xyz, matmul } from "./util.js"
import { get_obj_cfg_by_type } from "./obj_cfg.js"
import { updatecalibinfo } from "./calib.js"


var drawing = false;
var points = [];
var polyline;

var all_lines = [];

function to_polyline_attr(points) {
    return points.reduce(function (x, y) {
        return String(x) + "," + y;
    }
    )
}

function init_image_op() {
    var c = document.getElementById("maincanvas-wrapper");
    c.onclick = on_click;

    // var h = document.getElementById("resize-handle");
    // h.onmousedown = resize_mouse_down;

    c.onresize = on_resize;
    c.onmousemove = on_move;
}

function on_resize(ev) {
    console.log(ev);
}


function to_viewbox_coord(x, y) {
    var div = document.getElementById("maincanvas-svg");

    x = Math.round(x * 1920 / div.clientWidth);
    y = Math.round(y * 1080 / div.clientHeight);
    return [x, y];

}

function to_viewbox_coord2(x, y) {
    var div = document.getElementById("maincanvas-svg").getBoundingClientRect();
    var vp = document.getElementById("svg-image").getBoundingClientRect();
    
    var x0=vp.left - div.left;
    var y0=vp.top - div.top;

    var x1 = x - x0;
    var y1 = y - y0;

    x = Math.round(x1 * 1920 / vp.width);
    y = Math.round(y1 * 1080 / vp.height);
    return [x, y];

}

function center_crop(x, y) {
    var img = data.world.images.active_image(); //document.getElementById("camera");
    if (img && (img.naturalWidth > 0)) {
        x = x / 1920 * img.naturalWidth;
        y = y / 1080 * img.naturalHeight;
        clear_canvas();
        var c = document.getElementById("canvas");
        var ctx = c.getContext("2d");
        ctx.lineWidth = 0.5;

        // note: 320*240 should be adjustable
        var targetWidth = img.naturalWidth / 10;
        var targetHeight = img.naturalHeight / 10;
        var crop_area = [
            x - targetWidth / 2,
            y - targetHeight / 2,
            targetWidth,
            targetHeight
        ];

        ctx.drawImage(img, crop_area[0], crop_area[1], crop_area[2], crop_area[3], 0, 0, ctx.canvas.width, ctx.canvas.height);// ctx.canvas.clientHeight);
        //ctx.drawImage(img, 0,0,img.naturalWidth, img.naturalHeight, 0, 0, 320, 180);// ctx.canvas.clientHeight);
        // var imgfinal = vectorsub(imgfinal, [crop_area[0],crop_area[1]]);
        // var trans_ratio = {
        //     x: ctx.canvas.height/crop_area[3],
        //     y: ctx.canvas.height/crop_area[3],
        // }

        // draw_box_on_image(ctx, box, imgfinal, trans_ratio, true);
        ctx.strokeStyle = "#ff00ff";
        ctx.beginPath();
        ctx.moveTo(ctx.canvas.width / 2, ctx.canvas.height / 2 - 10);
        ctx.lineTo(ctx.canvas.width / 2, ctx.canvas.height / 2 + 10);
        ctx.moveTo(ctx.canvas.width / 2 - 10, ctx.canvas.height / 2);
        ctx.lineTo(ctx.canvas.width / 2 + 10, ctx.canvas.height / 2);
        ctx.stroke();
        ctx.closePath();

    }
}

function on_move(e) {
    var p = to_viewbox_coord2(e.layerX, e.layerY);
    var x = p[0];
    var y = p[1];

    console.log(x, y);
    if (e.ctrlKey) {
        center_crop(x, y);
    }
    if (drawing) {
        polyline.setAttribute("points", to_polyline_attr(points) + ',' + x + ',' + y);
    }

}

function on_click(e) {
    var p = to_viewbox_coord2(e.layerX, e.layerY);
    var x = p[0];
    var y = p[1];
    console.log(x, y);


    if (!drawing) {

        if (e.ctrlKey) {
            drawing = true;
            var svg = document.getElementById("svg-polys");
            //svg.style.position = "absolute";

            polyline = document.createElementNS("http://www.w3.org/2000/svg", 'polyline');
            svg.appendChild(polyline);
            points.push(x);
            points.push(y);


            polyline.setAttribute("class", "maincanvas-line")
            polyline.setAttribute("points", to_polyline_attr(points));

            var c = document.getElementById("maincanvas-wrapper");
            c.onmousemove = on_move;
            c.ondblclick = on_dblclick;
            // c.onkeydown = on_key;   

        }

    } else {
        if (points[points.length - 2] != x || points[points.length - 1] != y) {
            points.push(x);
            points.push(y);
            polyline.setAttribute("points", to_polyline_attr(points));

        }
        if (points.length == 8) {
            on_dblclick(e);
        }

    }




    function on_dblclick(e) {

        points.push(points[0]);
        points.push(points[1]);

        polyline.setAttribute("points", to_polyline_attr(points));
        console.log(points)

        all_lines.push(points);
        var poly = { poly: { points: points, cam: data.world.images.active_name }, obj_type: "Plane", obj_id: "1000" }
        data.world.polygons.push(poly);

        drawing = false;
        points = [];

        var c = document.getElementById("maincanvas-wrapper");
        // c.onmousemove = null;
        c.ondblclick = null;
        c.onkeypress = null;
        c.blur();
    }



    // function cancel(){

    //         polyline.remove();

    //         data.world.polygons=[];

    //         drawing = false;
    //         points = [];
    //         var c = document.getElementById("maincanvas-wrapper");
    //         c.onmousemove = null;
    //         c.ondblclick = null;
    //         c.onkeypress = null;

    //         c.blur();
    // }

    // function on_key(e){
    //     console.log(e);
    //     if (e.key == "Escape"){

    //         cancel();

    //     }
    // }
}

function empty_polys() {
    var polys = data.world.polygons;
    for (var k in polys) {
        if (polys[k].poly.cam == data.world.images.active_name) {
            console.log(polys[k]);
            delete polys[k];
        }
    }
    render_2d_image();

}


// all boxes
function clear_main_canvas() {

    //var c = document.getElementById("maincanvas");
    //var ctx = c.getContext("2d");

    //ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

    var boxes = document.getElementById("svg-boxes").children;

    if (boxes.length > 0) {
        for (var c = boxes.length - 1; c >= 0; c--) {
            boxes[c].remove();
        }
    }

    var polys = document.getElementById("svg-polys").children;

    if (polys.length > 0) {
        for (var c = polys.length - 1; c >= 0; c--) {
            polys[c].remove();
        }
    }

}


function get_active_calib() {
    var scene_meta = data.meta.find(function (x) { return x.scene == data.world.file_info.scene; });


    if (!scene_meta.calib) {
        return null;
    }

    var active_image_name = data.world.images.active_name;
    var calib = scene_meta.calib[active_image_name];

    return calib;
}


function choose_best_camera_for_point(x, y, z) {
    var scene_meta = data.meta.find(function (x) { return x.scene == data.world.file_info.scene; });


    if (!scene_meta.calib) {
        return null;
    }

    var proj_pos = [];
    for (var i in scene_meta.calib) {
        var imgpos = matmul(scene_meta.calib[i].extrinsic, [x, y, z, 1], 4);
        proj_pos.push({ calib: i, pos: vector4to3(imgpos) });
    }

    var valid_proj_pos = proj_pos.filter(function (p) {
        return all_points_in_image_range(p.pos);
    });

    valid_proj_pos.forEach(function (p) {
        p.dist_to_center = p.pos[0] * p.pos[0] + p.pos[1] * p.pos[1];
    });

    valid_proj_pos.sort(function (x, y) {
        return x.dist_to_center - y.dist_to_center;
    });

    //console.log(valid_proj_pos);

    if (valid_proj_pos.length > 0) {
        return valid_proj_pos[0].calib;
    }

    return null;

}

function get_trans_ratio() {
    var img = data.world.images.active_image();

    if (!img || img.width == 0) {
        return null;
    }

    var clientWidth, clientHeight;

    clientWidth = 1920;
    clientHeight = 1080;

    var trans_ratio = {
        x: clientWidth / img.naturalWidth,
        y: clientHeight / img.naturalHeight,
    };

    return trans_ratio;
}

function show_image() {
    var svgimage = document.getElementById("svg-image");

    // active img is set by global, it's not set sometimes.
    var img = data.world.images.active_image();
    if (img) {
        svgimage.setAttribute("xlink:href", img.src);
    }
}

function render_2d_image() {
    console.log("2d image rendered!");
    updatecalibinfo();



    clear_main_canvas();

    if (params["hide image"]) {
        hide_canvas();
        return;
    }

    show_image();

    draw_svg();

    function hide_canvas() {
        //document.getElementsByClassName("ui-wrapper")[0].style.display="none";
        document.getElementById("maincanvas-wrapper").style.display = "none";
    }

    function show_canvas() {
        document.getElementById("maincanvas-wrapper").style.display = "inline";
    }

    function draw_svg() {
        // draw picture
        var img = data.world.images.active_image();

        if (!img || img.width == 0) {
            hide_canvas();
            return;
        }

        show_canvas();

        var trans_ratio = get_trans_ratio();

        var calib = get_active_calib();
        if (!calib) {
            return;
        }

        var boxessvg = document.getElementById("svg-boxes");

        // draw boxes
        data.world.boxes.forEach(function (box) {
            var imgfinal = box_to_2d_points(box, calib);
            if (imgfinal) {
                var box_svg = box_to_svg(box, imgfinal, trans_ratio, selected_box == box);
                boxessvg.appendChild(box_svg);
            }

        });

        var polyssvg = document.getElementById("svg-polys");

        var polys = data.world.polygons;
        for (var k in polys) {
            if (polys[k].poly.cam == data.world.images.active_name) {
                console.log(polys[k]);
                var poly_svg = document.createElementNS("http://www.w3.org/2000/svg", 'polygon');
                polyssvg.appendChild(poly_svg);
                poly_svg.setAttribute("class", "maincanvas-line")
                poly_svg.setAttribute("points",
                    polys[k].poly.points.reduce(function (x, y) {
                        return String(x) + "," + y;
                    })
                )
                var circle = document.createElementNS("http://www.w3.org/2000/svg", 'circle');
                polyssvg.appendChild(circle);
                circle.setAttribute("r", "10");
                // circle.setAttribute("fill", "red");
                circle.setAttribute("stroke","pink");
                // circle.setAttribute("stroke-width","1");
                circle.setAttribute("cx", Math.round(polys[k].poly.points[0] * trans_ratio.x));
                circle.setAttribute("cy", Math.round(polys[k].poly.points[1] * trans_ratio.y));

            }
        }
        var points = data.world.points;
        var pos_array = points.geometry.getAttribute("position").array;
        var relative_position = [];
        var indices = [];

        data.world.boxes.forEach(function (box) {
            // console.log(data.world._get_points_index_of_box(points, box, 1.0));
            indices = indices.concat(data.world._get_points_index_of_box(points, box, 1.0));

        });
        // return;
        // var imgposs = [];
        indices.forEach(function (i) {
            // if (i % 3 != 0) { return; }
            //for (var i  = 0; i < pos.count; i++){
            var x = pos_array[i * 3];
            // if (x > 0) { return; }
            var y = pos_array[i * 3 + 1];
            var z = pos_array[i * 3 + 2];
            var imgpos = matmul(calib.extrinsic, [x, y, z, 1], 4);
            var imgpos3 = vector4to3(imgpos);
            var imgpos2;
            if (calib.intrinsic.length > 9) {
                imgpos2 = matmul(calib.intrinsic, imgpos, 4);
            } else {
                imgpos2 = matmul(calib.intrinsic, imgpos3, 3);
            }
            var imgfinal = vector3_nomalize(imgpos2);

            if (imgfinal[0] > img.width || imgfinal[0] < 0 || imgfinal[1] > img.height || imgfinal[1] < 0) {
                return;
            }
            var circle = document.createElementNS("http://www.w3.org/2000/svg", 'circle');
            polyssvg.appendChild(circle);
            circle.setAttribute("r", "1");
            circle.setAttribute("fill", "red");
            circle.setAttribute("stroke","red");
            circle.setAttribute("stroke-width","1");
            circle.setAttribute("cx", Math.round(imgfinal[0] * trans_ratio.x));
            circle.setAttribute("cy", Math.round(imgfinal[1] * trans_ratio.y));
            // console.log(imgfinal);
            // imgposs.push([imgfinal]);
            // var p = [x-center.x, y-center.y, z-center.z, 1];
            // var tp = matmul(trans, p, 4);
            // relative_position.push([tp[0],tp[1],tp[2]]);
        });


    }

}

function box_to_2d_points(box, calib) {
    var scale = box.scale;
    var pos = box.position;
    var rotation = box.rotation;

    var box3d = psr_to_xyz(pos, scale, rotation);

    var imgpos = matmul(calib.extrinsic, box3d, 4);

    if (calib.rect) {
        imgpos = matmul(calib.rect, imgpos, 4);
    }

    var imgpos3 = vector4to3(imgpos);

    var imgpos2;
    if (calib.intrinsic.length > 9) {
        imgpos2 = matmul(calib.intrinsic, imgpos, 4);
    } else {
        imgpos2 = matmul(calib.intrinsic, imgpos3, 3);
    }

    if (!all_points_in_image_range(imgpos3)) {
        return null;
    }
    var imgfinal = vector3_nomalize(imgpos2);

    return imgfinal;
}

function box_to_svg(box, box_corners, trans_ratio, selected) {


    var imgfinal = box_corners.map(function (x, i) {
        if (i % 2 == 0) {
            return Math.round(x * trans_ratio.x);
        } else {
            return Math.round(x * trans_ratio.y);
        }
    })


    var svg = document.createElementNS("http://www.w3.org/2000/svg", 'g');
    svg.setAttribute("id", "svg-box-local-" + box.obj_local_id);

    if (selected) {
        svg.setAttribute("class", box.obj_type + " box-svg-selected");
    } else {
        svg.setAttribute("class", box.obj_type);
    }

    var front_panel = document.createElementNS("http://www.w3.org/2000/svg", 'polygon');
    svg.appendChild(front_panel);
    front_panel.setAttribute("points",
        imgfinal.slice(0, 4 * 2).reduce(function (x, y) {
            return String(x) + "," + y;
        })
    )

    var circle = document.createElementNS("http://www.w3.org/2000/svg", 'circle');
    svg.appendChild(circle);
    circle.setAttribute("r", "5");
    // circle.setAttribute("fill", "red");
    circle.setAttribute("stroke","yellow");
    // circle.setAttribute("stroke-width","1");
    circle.setAttribute("cx", imgfinal[6]);
    circle.setAttribute("cy", imgfinal[7]);
    // circle.setAttribute("cx", (imgfinal[0]+imgfinal[6])/2);
    // circle.setAttribute("cy", (imgfinal[1]+imgfinal[7])/2);

    /*
    var back_panel =  document.createElementNS("http://www.w3.org/2000/svg", 'polygon');
    svg.appendChild(back_panel);
    back_panel.setAttribute("points",
        imgfinal.slice(4*2).reduce(function(x,y){            
            return String(x)+","+y;
        })
    )
    */

    for (var i = 0; i < 4; ++i) {
        var line = document.createElementNS("http://www.w3.org/2000/svg", 'line');
        svg.appendChild(line);
        line.setAttribute("x1", imgfinal[(4 + i) * 2]);
        line.setAttribute("y1", imgfinal[(4 + i) * 2 + 1]);
        line.setAttribute("x2", imgfinal[(4 + (i + 1) % 4) * 2]);
        line.setAttribute("y2", imgfinal[(4 + (i + 1) % 4) * 2 + 1]);
    }


    for (var i = 0; i < 4; ++i) {
        var line = document.createElementNS("http://www.w3.org/2000/svg", 'line');
        svg.appendChild(line);
        line.setAttribute("x1", imgfinal[i * 2]);
        line.setAttribute("y1", imgfinal[i * 2 + 1]);
        line.setAttribute("x2", imgfinal[(i + 4) * 2]);
        line.setAttribute("y2", imgfinal[(i + 4) * 2 + 1]);
    }

    return svg;
}


function draw_box_on_image(ctx, box, box_corners, trans_ratio, selected) {
    var imgfinal = box_corners;

    if (!selected) {
        ctx.strokeStyle = get_obj_cfg_by_type(box.obj_type).color;

        var c = get_obj_cfg_by_type(box.obj_type).color;
        var r = "0x" + c.slice(1, 3);
        var g = "0x" + c.slice(3, 5);
        var b = "0x" + c.slice(5, 7);

        ctx.fillStyle = "rgba(" + parseInt(r) + "," + parseInt(g) + "," + parseInt(b) + ",0.2)";
    }
    else {
        ctx.strokeStyle = "#ff00ff";
        ctx.fillStyle = "rgba(255,0,255,0.2)";
    }

    // front panel
    ctx.beginPath();
    ctx.moveTo(imgfinal[3 * 2] * trans_ratio.x, imgfinal[3 * 2 + 1] * trans_ratio.y);

    for (var i = 0; i < imgfinal.length / 2 / 2; i++) {
        ctx.lineTo(imgfinal[i * 2 + 0] * trans_ratio.x, imgfinal[i * 2 + 1] * trans_ratio.y);
    }

    ctx.closePath();
    ctx.fill();

    // frame
    ctx.beginPath();

    ctx.moveTo(imgfinal[3 * 2] * trans_ratio.x, imgfinal[3 * 2 + 1] * trans_ratio.y);

    for (var i = 0; i < imgfinal.length / 2 / 2; i++) {
        ctx.lineTo(imgfinal[i * 2 + 0] * trans_ratio.x, imgfinal[i * 2 + 1] * trans_ratio.y);
    }
    //ctx.stroke();


    //ctx.strokeStyle="#ff00ff";
    //ctx.beginPath();

    ctx.moveTo(imgfinal[7 * 2] * trans_ratio.x, imgfinal[7 * 2 + 1] * trans_ratio.y);

    for (var i = 4; i < imgfinal.length / 2; i++) {
        ctx.lineTo(imgfinal[i * 2 + 0] * trans_ratio.x, imgfinal[i * 2 + 1] * trans_ratio.y);
    }

    ctx.moveTo(imgfinal[0 * 2] * trans_ratio.x, imgfinal[0 * 2 + 1] * trans_ratio.y);
    ctx.lineTo(imgfinal[4 * 2 + 0] * trans_ratio.x, imgfinal[4 * 2 + 1] * trans_ratio.y);
    ctx.moveTo(imgfinal[1 * 2] * trans_ratio.x, imgfinal[1 * 2 + 1] * trans_ratio.y);
    ctx.lineTo(imgfinal[5 * 2 + 0] * trans_ratio.x, imgfinal[5 * 2 + 1] * trans_ratio.y);
    ctx.moveTo(imgfinal[2 * 2] * trans_ratio.x, imgfinal[2 * 2 + 1] * trans_ratio.y);
    ctx.lineTo(imgfinal[6 * 2 + 0] * trans_ratio.x, imgfinal[6 * 2 + 1] * trans_ratio.y);
    ctx.moveTo(imgfinal[3 * 2] * trans_ratio.x, imgfinal[3 * 2 + 1] * trans_ratio.y);
    ctx.lineTo(imgfinal[7 * 2 + 0] * trans_ratio.x, imgfinal[7 * 2 + 1] * trans_ratio.y);


    ctx.stroke();
}


function clear_canvas() {
    var c = document.getElementById("canvas");
    var ctx = c.getContext("2d");

    ctx.clearRect(0, 0, canvas.width, canvas.height);
}


function all_points_in_image_range(p) {
    for (var i = 0; i < p.length / 3; i++) {
        if (p[i * 3 + 2] < 0) {
            return false;
        }
    }

    return true;
}


// draw highlighed box
function update_image_box_projection(box) {
    var scene_meta = data.meta.find(function (x) { return x.scene == data.world.file_info.scene; });

    var active_image_name = data.world.images.active_name;
    if (!scene_meta.calib) {
        return;
    }

    var calib = scene_meta.calib[active_image_name]
    if (!calib) {
        return;
    }

    if (calib) {
        var scale = box.scale;
        var pos = box.position;
        var rotation = box.rotation;

        var img = data.world.images.active_image(); //document.getElementById("camera");
        if (img && (img.naturalWidth > 0)) {

            clear_canvas();


            var imgfinal = box_to_2d_points(box, calib)


            if (imgfinal != null) {  // if projection is out of range of the image, stop drawing.

                var c = document.getElementById("canvas");
                var ctx = c.getContext("2d");
                ctx.lineWidth = 0.5;

                // note: 320*240 should be adjustable
                var crop_area = crop_image(img.naturalWidth, img.naturalHeight, ctx.canvas.width, ctx.canvas.height, imgfinal);

                ctx.drawImage(img, crop_area[0], crop_area[1], crop_area[2], crop_area[3], 0, 0, ctx.canvas.width, ctx.canvas.height);// ctx.canvas.clientHeight);
                //ctx.drawImage(img, 0,0,img.naturalWidth, img.naturalHeight, 0, 0, 320, 180);// ctx.canvas.clientHeight);
                var imgfinal = vectorsub(imgfinal, [crop_area[0], crop_area[1]]);
                var trans_ratio = {
                    x: ctx.canvas.height / crop_area[3],
                    y: ctx.canvas.height / crop_area[3],
                }

                draw_box_on_image(ctx, box, imgfinal, trans_ratio, true);
            }
        }
    }
}



function crop_image(imgWidth, imgHeight, clientWidth, clientHeight, corners) {
    var maxx = 0, maxy = 0, minx = imgWidth, miny = imgHeight;

    for (var i = 0; i < corners.length / 2; i++) {
        var x = corners[i * 2];
        var y = corners[i * 2 + 1];

        if (x > maxx) maxx = x;
        else if (x < minx) minx = x;

        if (y > maxy) maxy = y;
        else if (y < miny) miny = y;
    }

    var targetWidth = (maxx - minx) * 1.5;
    var targetHeight = (maxy - miny) * 1.5;

    if (targetWidth / targetHeight > clientWidth / clientHeight) {
        //increate height
        targetHeight = targetWidth * clientHeight / clientWidth;
    }
    else {
        targetWidth = targetHeight * clientWidth / clientHeight;
    }

    var centerx = (maxx + minx) / 2;
    var centery = (maxy + miny) / 2;

    return [
        centerx - targetWidth / 2,
        centery - targetHeight / 2,
        targetWidth,
        targetHeight
    ];
}

function vectorsub(vs, v) {
    var ret = [];
    var vl = v.length;

    for (var i = 0; i < vs.length / vl; i++) {
        for (var j = 0; j < vl; j++)
            ret[i * vl + j] = vs[i * vl + j] - v[j];
    }

    return ret;
}


var image_manager = {
    display_image: function () {
        render_2d_image();
    },

    add_box: function (box) {
        var calib = get_active_calib();
        if (!calib) {
            return;
        }
        var trans_ratio = get_trans_ratio();
        if (trans_ratio) {
            var imgfinal = box_to_2d_points(box, calib);
            if (imgfinal) {
                var imgfinal = imgfinal.map(function (x, i) {
                    if (i % 2 == 0) {
                        return Math.round(x * trans_ratio.x);
                    } else {
                        return Math.round(x * trans_ratio.y);
                    }
                })

                var svg_box = box_to_svg(box, imgfinal, trans_ratio);
                var svg = document.getElementById("svg-boxes");
                svg.appendChild(svg_box);
            }
        }
    },


    select_bbox: function (box_obj_local_id, obj_type) {
        var b = document.getElementById("svg-box-local-" + box_obj_local_id);
        if (b) {
            b.setAttribute("class", "box-svg-selected");
        }
    },


    unselect_bbox: function (box_obj_local_id, obj_type) {
        var b = document.getElementById("svg-box-local-" + box_obj_local_id);

        if (b)
            b.setAttribute("class", obj_type);
    },

    remove_box: function (box_obj_local_id) {
        var b = document.getElementById("svg-box-local-" + box_obj_local_id);

        if (b)
            b.remove();
    },

    update_obj_type: function (box_obj_local_id, obj_type) {
        this.select_bbox(box_obj_local_id, obj_type);
    },

    update_box: function (box) {
        var b = document.getElementById("svg-box-local-" + box.obj_local_id);
        if (!b) {
            return;
        }

        var children = b.childNodes;

        var calib = get_active_calib();
        if (!calib) {
            return;
        }

        var trans_ratio = get_trans_ratio();
        var imgfinal = box_to_2d_points(box, calib);

        if (!imgfinal) {
            //box may go out of image
            return;
        }
        var imgfinal = imgfinal.map(function (x, i) {
            if (i % 2 == 0) {
                return Math.round(x * trans_ratio.x);
            } else {
                return Math.round(x * trans_ratio.y);
            }
        })

        if (imgfinal) {
            var front_panel = children[0];
            front_panel.setAttribute("points",
                imgfinal.slice(0, 4 * 2).reduce(function (x, y) {
                    return String(x) + "," + y;
                })
            )
            
            var circle = children[1];
            circle.setAttribute("cx", imgfinal[6]);
            circle.setAttribute("cy", imgfinal[7]);


            for (var i = 0; i < 4; ++i) {
                var line = children[2 + i];
                line.setAttribute("x1", imgfinal[(4 + i) * 2]);
                line.setAttribute("y1", imgfinal[(4 + i) * 2 + 1]);
                line.setAttribute("x2", imgfinal[(4 + (i + 1) % 4) * 2]);
                line.setAttribute("y2", imgfinal[(4 + (i + 1) % 4) * 2 + 1]);
            }


            for (var i = 0; i < 4; ++i) {
                var line = children[6 + i];
                line.setAttribute("x1", imgfinal[i * 2]);
                line.setAttribute("y1", imgfinal[i * 2 + 1]);
                line.setAttribute("x2", imgfinal[(i + 4) * 2]);
                line.setAttribute("y2", imgfinal[(i + 4) * 2 + 1]);
            }
        }

    }
}

export { init_image_op, render_2d_image, update_image_box_projection, clear_canvas, clear_main_canvas, choose_best_camera_for_point, image_manager, empty_polys }