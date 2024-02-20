translate(v=[0,0,0]) {
import("../scad-files/booleans/extention_by_longer.stl");
}

translate(v=[-8,0,0]) {
import("../scad-files/booleans/extention_by_smaller.stl");
}


translate(v=[8,0,0]) {
import("../scad-files/booleans/shifted_in_space.stl");
}



translate(v=[2.5,0,0]) {
    import("../scad-files/booleans/one_with_other.stl");
}

translate(v=[-2.5 ,0,0]) {
    import("../scad-files/booleans/one_with_other_overlap.stl");
}

translate(v=[5,0,0]) {
import("../scad-files/booleans/bigger_by_smaller.stl");
}

translate(v=[-5,0,0]) {
import("../scad-files/booleans/shifted_in_plane.stl");
}