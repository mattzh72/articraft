from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_modular_synth_rack")

    frame_mat = model.material("black_anodized_aluminum", rgba=(0.015, 0.016, 0.018, 1.0))
    hinge_mat = model.material("brushed_steel_hinge", rgba=(0.58, 0.56, 0.52, 1.0))
    screw_mat = model.material("dark_screw_heads", rgba=(0.04, 0.04, 0.045, 1.0))
    panel_mats = (
        model.material("warm_silver_panel", rgba=(0.78, 0.76, 0.70, 1.0)),
        model.material("matte_graphite_panel", rgba=(0.12, 0.13, 0.15, 1.0)),
        model.material("off_white_panel", rgba=(0.86, 0.84, 0.78, 1.0)),
    )
    label_mat = model.material("screen_printed_labels", rgba=(0.02, 0.02, 0.022, 1.0))
    jack_metal_mat = model.material("nickel_jack_bushings", rgba=(0.72, 0.70, 0.64, 1.0))
    jack_hole_mat = model.material("black_jack_sockets", rgba=(0.0, 0.0, 0.0, 1.0))
    knob_mat = model.material("black_knob_caps", rgba=(0.018, 0.018, 0.018, 1.0))
    pointer_mat = model.material("white_knob_pointers", rgba=(0.92, 0.90, 0.82, 1.0))

    rack_height = 0.78
    rack_width = 0.56
    rack_depth = 0.070
    rail_thickness = 0.040
    panel_width = 0.135
    panel_height = 0.680
    panel_thickness = 0.008
    hinge_y = 0.040
    hinge_lines = (-0.225, -0.070, 0.085)

    frame = model.part("rack_frame")
    frame.visual(
        Box((rack_width, rack_depth, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, rack_height / 2.0 - 0.025)),
        material=frame_mat,
        name="top_rail",
    )
    frame.visual(
        Box((rack_width, rack_depth, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -rack_height / 2.0 + 0.025)),
        material=frame_mat,
        name="bottom_rail",
    )
    frame.visual(
        Box((rail_thickness, rack_depth, rack_height)),
        origin=Origin(xyz=(-rack_width / 2.0 + rail_thickness / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="side_rail_0",
    )
    frame.visual(
        Box((rail_thickness, rack_depth, rack_height)),
        origin=Origin(xyz=(rack_width / 2.0 - rail_thickness / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="side_rail_1",
    )
    frame.visual(
        Box((rack_width - 0.070, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.034, 0.0)),
        material=frame_mat,
        name="rear_cable_tie_bar",
    )

    for index, hinge_x in enumerate(hinge_lines):
        frame.visual(
            Box((0.012, 0.018, panel_height + 0.020)),
            origin=Origin(xyz=(hinge_x - 0.012, hinge_y - 0.005, 0.0)),
            material=hinge_mat,
            name=f"hinge_leaf_{index}",
        )
        frame.visual(
            Box((0.010, 0.050, panel_height + 0.020)),
            origin=Origin(xyz=(hinge_x - 0.022, 0.004, 0.0)),
            material=frame_mat,
            name=f"hinge_post_{index}",
        )

    for x in (-rack_width / 2.0 + 0.018, rack_width / 2.0 - 0.018):
        for z in (-0.290, -0.150, 0.150, 0.290):
            frame.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(xyz=(x, 0.037, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=screw_mat,
                name=f"rack_screw_{x:.2f}_{z:.2f}",
            )

    knob_specs = (
        ((0.042, 0.205), (0.100, 0.205)),
        ((0.044, 0.205), (0.102, 0.205)),
        ((0.042, 0.205), (0.100, 0.205)),
    )

    for index, hinge_x in enumerate(hinge_lines):
        panel = model.part(f"module_{index}")
        panel.visual(
            Box((panel_width, panel_thickness, panel_height)),
            origin=Origin(xyz=(panel_width / 2.0, 0.0, 0.0)),
            material=panel_mats[index],
            name="face",
        )
        panel.visual(
            Cylinder(radius=0.005, length=panel_height),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=hinge_mat,
            name="hinge_barrel",
        )
        panel.visual(
            Box((0.018, 0.010, panel_height)),
            origin=Origin(xyz=(0.010, 0.0, 0.0)),
            material=hinge_mat,
            name="hinge_reinforcement",
        )
        panel.visual(
            Box((panel_width - 0.030, 0.0015, 0.028)),
            origin=Origin(xyz=(panel_width / 2.0 + 0.006, panel_thickness / 2.0 + 0.0007, 0.292)),
            material=label_mat,
            name="label_strip",
        )
        panel.visual(
            Box((0.020, 0.010, 0.070)),
            origin=Origin(xyz=(panel_width - 0.013, panel_thickness / 2.0 + 0.003, -0.252)),
            material=frame_mat,
            name="pull_tab",
        )

        for sx in (0.020, panel_width - 0.020):
            for sz in (-panel_height / 2.0 + 0.024, panel_height / 2.0 - 0.024):
                panel.visual(
                    Cylinder(radius=0.004, length=0.003),
                    origin=Origin(xyz=(sx, panel_thickness / 2.0 + 0.001, sz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                    material=screw_mat,
                    name=f"panel_screw_{sx:.2f}_{sz:.2f}",
                )

        for row, z in enumerate((-0.145, -0.205, -0.265)):
            for col, x in enumerate((0.045, 0.100)):
                panel.visual(
                    Cylinder(radius=0.0075, length=0.004),
                    origin=Origin(xyz=(x, panel_thickness / 2.0 + 0.0015, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                    material=jack_metal_mat,
                    name=f"jack_{row}_{col}_ring",
                )
                panel.visual(
                    Cylinder(radius=0.0038, length=0.0045),
                    origin=Origin(xyz=(x, panel_thickness / 2.0 + 0.0036, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                    material=jack_hole_mat,
                    name=f"jack_{row}_{col}_hole",
                )

        hinge = model.articulation(
            f"module_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=panel,
            origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=0.0, upper=1.75),
        )
        hinge.meta["description"] = "Vertical left-edge hinge; positive motion swings the module panel forward for cable access."

        for knob_index, (x, z) in enumerate(knob_specs[index]):
            knob = model.part(f"knob_{index}_{knob_index}")
            knob.visual(
                Cylinder(radius=0.017, length=0.004),
                origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=hinge_mat,
                name="collar",
            )
            knob.visual(
                Cylinder(radius=0.0135, length=0.012),
                origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=knob_mat,
                name="cap",
            )
            knob.visual(
                Box((0.0035, 0.0015, 0.017)),
                origin=Origin(xyz=(0.0, 0.0164, 0.0045)),
                material=pointer_mat,
                name="pointer",
            )
            model.articulation(
                f"knob_{index}_{knob_index}_shaft",
                ArticulationType.REVOLUTE,
                parent=panel,
                child=knob,
                origin=Origin(xyz=(x, panel_thickness / 2.0, z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=0.4, velocity=6.0, lower=-2.35, upper=2.35),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("rack_frame")
    module_hinges = [object_model.get_articulation(f"module_{i}_hinge") for i in range(3)]

    ctx.check(
        "three hinged module faces",
        len(module_hinges) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in module_hinges),
        details=f"hinges={[j.name for j in module_hinges]}",
    )

    for index, hinge in enumerate(module_hinges):
        panel = object_model.get_part(f"module_{index}")
        ctx.check(
            f"module_{index} hinge is vertical and left edged",
            hinge.axis == (0.0, 0.0, 1.0)
            and hinge.motion_limits is not None
            and hinge.motion_limits.lower == 0.0
            and hinge.motion_limits.upper is not None
            and hinge.motion_limits.upper >= 1.5,
            details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
        )

        face_aabb = ctx.part_element_world_aabb(panel, elem="face")
        hinge_x = hinge.origin.xyz[0]
        ctx.check(
            f"module_{index} panel face starts at hinge line",
            face_aabb is not None and abs(face_aabb[0][0] - hinge_x) < 0.002,
            details=f"face_aabb={face_aabb}, hinge_x={hinge_x}",
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="x",
            min_gap=0.0,
            max_gap=0.003,
            positive_elem="hinge_barrel",
            negative_elem=f"hinge_leaf_{index}",
            name=f"module_{index} hinge barrel sits beside fixed leaf",
        )

        rest_aabb = ctx.part_element_world_aabb(panel, elem="face")
        rest_front = rest_aabb[1][1] if rest_aabb is not None else None
        with ctx.pose({hinge: 1.20}):
            open_aabb = ctx.part_element_world_aabb(panel, elem="face")
            open_front = open_aabb[1][1] if open_aabb is not None else None
        ctx.check(
            f"module_{index} swings forward for cable access",
            rest_front is not None and open_front is not None and open_front > rest_front + 0.08,
            details=f"rest_front={rest_front}, open_front={open_front}",
        )

    knob_joints = [j for j in object_model.articulations if j.name.startswith("knob_")]
    ctx.check(
        "six separate rotary knobs",
        len(knob_joints) == 6 and all(j.articulation_type == ArticulationType.REVOLUTE for j in knob_joints),
        details=f"knob_joints={[j.name for j in knob_joints]}",
    )

    return ctx.report()


object_model = build_object_model()
