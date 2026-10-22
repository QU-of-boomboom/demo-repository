"""生成无外部模型下载的 Fortress 世界；三机由理想位姿驱动。"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def world_sdf(cell_m=1.):
    root = ET.Element("sdf", version="1.8")
    world = ET.SubElement(root, "world", name="warehouse")
    physics = ET.SubElement(world, "physics", name="ideal_clock", type="ignored")
    ET.SubElement(physics, "max_step_size").text = "0.01"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    for library, name in (("physics", "Physics"), ("user-commands", "UserCommands"),
                          ("scene-broadcaster", "SceneBroadcaster")):
        ET.SubElement(world, "plugin", filename=f"ignition-gazebo-{library}-system",
                      name=f"ignition::gazebo::systems::{name}")
    light = ET.SubElement(world, "light", name="sun", type="directional")
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.9 0.9 0.9 1"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"
    ET.SubElement(light, "cast_shadows").text = "true"

    def model(name, pose, size, color):
        node = ET.SubElement(world, "model", name=name)
        ET.SubElement(node, "static").text = "true"
        ET.SubElement(node, "pose").text = pose
        link = ET.SubElement(node, "link", name="body")
        visual = ET.SubElement(link, "visual", name="shape")
        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = size
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = color
        ET.SubElement(material, "diffuse").text = color
        return link

    model("floor", f"{1.5*cell_m} {2*cell_m} -0.05 0 0 0", f"{5*cell_m} {6*cell_m} 0.1", "0.18 0.23 0.3 1")
    for x in range(4):
        model(f"line_x{x}", f"{x*cell_m} {2*cell_m} 0.01 0 0 0", f"0.025 {4*cell_m} 0.02", "0.6 0.7 0.8 1")
    for y in range(5):
        model(f"line_y{y}", f"{1.5*cell_m} {y*cell_m} 0.01 0 0 0", f"{3*cell_m} 0.025 0.02", "0.6 0.7 0.8 1")
    for x in range(1, 4):
        for y in range(1, 5):
            model(f"box_{x}_{y}", f"{(x-.5)*cell_m} {(y-.5)*cell_m} 0.4 0 0 0",
                  f"{.8*cell_m} {.8*cell_m} 0.8", "0.7 0.42 0.16 1")
    for name, z, color in (("main", 1.5, "0.1 0.5 1 1"),
                            ("relay1", 1.8, "0.1 0.8 0.6 1"), ("relay2", 2.1, "0.7 0.9 0.2 1")):
        link = model(name, f"0 0 {z} 0 0 0", "0.22 0.16 0.08", color)
        for i, (x, y) in enumerate(((-.14, -.14), (-.14, .14), (.14, -.14), (.14, .14))):
            visual = ET.SubElement(link, "visual", name=f"rotor_{i}")
            ET.SubElement(visual, "pose").text = f"{x} {y} 0.04 0 0 0"
            cylinder = ET.SubElement(ET.SubElement(visual, "geometry"), "cylinder")
            ET.SubElement(cylinder, "radius").text = "0.095"
            ET.SubElement(cylinder, "length").text = "0.015"
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color
    ET.indent(root)
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("worlds/warehouse.sdf"))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(world_sdf(), encoding="utf-8")


if __name__ == "__main__":
    main()
