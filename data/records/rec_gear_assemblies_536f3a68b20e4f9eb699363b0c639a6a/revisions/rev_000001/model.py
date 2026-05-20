from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHAFT_LAYOUT = {
    "input": (-0.19, 0.35),
    "counter": (0.0, 0.23),
    "output": (0.19, 0.35),
}


def _bearing_block_shape(
    *,
    width: float,
    depth: float,
    height: float,
    bore_z: float,
    bore_radius: float,
    boss_radius: float,
) -> cq.Workplane:
    """Pillow-block bearing with a true through-bore along local Y."""
    body = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    boss = (
        cq.Workplane("XZ")
        .center(0.0, bore_z)
        .circle(boss_radius)
        .extrude((depth + 0.018) / 2.0, both=True)
    )
    bore = (
        cq.Workplane("XZ")
        .center(0.0, bore_z)
        .circle(bore_radius)
        .extrude((depth + 0.040) / 2.0, both=True)
    )
    return body.union(boss).cut(bore)


def _bearing_bush_shape(*, thickness: float, outer_radius: float, inner_radius: float, bore_z: float) -> cq.Workplane:
    """Thin bronze annular insert on the face of a bearing block."""
    return (
        cq.Workplane("XZ")
        .center(0.0, bore_z)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness / 2.0, both=True)
    )


def _spur_gear_mesh(name: str, teeth: int = 20):
    gear = SpurGear(
        module=0.010,
        teeth_number=teeth,
        width=0.055,
        backlash=0.0004,
    )
    body = gear.build(
        bore_d=0.046,
        hub_d=0.074,
        hub_length=0.074,
        n_spokes=4,
        spoke_width=0.012,
        spokes_id=0.052,
        spokes_od=0.150,
        chamfer=0.0015,
    )
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_transfer_gearbox")

    frame_blue = model.material("painted_frame_blue", rgba=(0.10, 0.18, 0.28, 1.0))
    block_gray = model.material("bearing_cast_iron", rgba=(0.42, 0.43, 0.42, 1.0))
    dark_steel = model.material("oiled_dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    bright_steel = model.material("bright_turned_steel", rgba=(0.64, 0.67, 0.69, 1.0))
    bronze = model.material("bronze_bushings", rgba=(0.79, 0.55, 0.24, 1.0))
    gear_steel = model.material("machined_gear_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    gear_bronze = model.material("counter_gear_bronze", rgba=(0.70, 0.49, 0.20, 1.0))

    upper_block_mesh = mesh_from_cadquery(
        _bearing_block_shape(
            width=0.120,
            depth=0.120,
            height=0.340,
            bore_z=SHAFT_LAYOUT["input"][1] - 0.060,
            bore_radius=0.034,
            boss_radius=0.066,
        ),
        "upper_bearing_block",
        tolerance=0.0008,
    )
    lower_block_mesh = mesh_from_cadquery(
        _bearing_block_shape(
            width=0.120,
            depth=0.120,
            height=0.250,
            bore_z=SHAFT_LAYOUT["counter"][1] - 0.060,
            bore_radius=0.036,
            boss_radius=0.068,
        ),
        "counter_bearing_block",
        tolerance=0.0008,
    )
    upper_bush_mesh = mesh_from_cadquery(
        _bearing_bush_shape(
            thickness=0.014,
            outer_radius=0.047,
            inner_radius=0.029,
            bore_z=SHAFT_LAYOUT["input"][1] - 0.060,
        ),
        "upper_bearing_bush",
        tolerance=0.0008,
    )
    lower_bush_mesh = mesh_from_cadquery(
        _bearing_bush_shape(
            thickness=0.014,
            outer_radius=0.049,
            inner_radius=0.031,
            bore_z=SHAFT_LAYOUT["counter"][1] - 0.060,
        ),
        "counter_bearing_bush",
        tolerance=0.0008,
    )
    gear_mesh = _spur_gear_mesh("shaft_spur_gear")
    counter_gear_mesh = _spur_gear_mesh("countershaft_spur_gear", teeth=20)

    frame = model.part("frame")
    frame.visual(
        Box((0.68, 0.90, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=frame_blue,
        name="open_base_pan",
    )
    for y in (-0.405, 0.405):
        frame.visual(
            Box((0.72, 0.060, 0.080)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material=frame_blue,
            name=f"side_rail_{0 if y < 0.0 else 1}",
        )
    for x in (-0.245, 0.0, 0.245):
        frame.visual(
            Box((0.050, 0.86, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.090)),
            material=frame_blue,
            name=f"crossmember_{x:+.3f}".replace("+", "p").replace("-", "m").replace(".", "_"),
        )

    bearing_y_positions = (-0.325, 0.325)
    for shaft_name, (x, z) in SHAFT_LAYOUT.items():
        is_counter = shaft_name == "counter"
        block_mesh = lower_block_mesh if is_counter else upper_block_mesh
        bush_mesh = lower_bush_mesh if is_counter else upper_bush_mesh
        shaft_radius = 0.026 if is_counter else 0.024
        for side_index, y in enumerate(bearing_y_positions):
            frame.visual(
                block_mesh,
                origin=Origin(xyz=(x, y, 0.060)),
                material=block_gray,
                name=f"{shaft_name}_bearing_{side_index}",
            )
            frame.visual(
                Box((shaft_radius * 1.45, 0.048, 0.016)),
                origin=Origin(xyz=(x, y, z - shaft_radius - 0.006)),
                material=bronze,
                name=f"{shaft_name}_lower_shell_{side_index}",
            )
            for face_index, face_offset in enumerate((-0.067, 0.067)):
                frame.visual(
                    bush_mesh,
                    origin=Origin(xyz=(x, y + face_offset, 0.060)),
                    material=bronze,
                    name=f"{shaft_name}_bush_{side_index}_{face_index}",
                )
        frame.visual(
            Box((0.034, 0.70, 0.025)),
            origin=Origin(xyz=(x, 0.0, z - 0.130)),
            material=dark_steel,
            name=f"{shaft_name}_oil_sling",
        )

    def add_rotating_shaft(part_name: str, gear_material, shaft_radius: float, gear_mesh_obj) -> None:
        shaft_x, shaft_z = SHAFT_LAYOUT[part_name.split("_")[0]]
        shaft_part = model.part(part_name)
        shaft_part.visual(
            Cylinder(radius=shaft_radius, length=0.865),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name=f"{part_name}_barrel",
        )
        shaft_part.visual(
            gear_mesh_obj,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gear_material,
            name=f"{part_name}_spur_gear",
        )
        for y in (-0.235, 0.235):
            shaft_part.visual(
                Cylinder(radius=shaft_radius + 0.011, length=0.026),
                origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_steel,
                name=f"{part_name}_collar_{0 if y < 0.0 else 1}",
            )
        shaft_part.visual(
            Box((shaft_radius * 0.95, 0.115, shaft_radius * 0.34)),
            origin=Origin(xyz=(0.0, 0.0, shaft_radius + 0.002)),
            material=dark_steel,
            name=f"{part_name}_key",
        )
        model.articulation(
            f"{part_name}_joint",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=shaft_part,
            origin=Origin(xyz=(shaft_x, 0.0, shaft_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=18.0, lower=-math.pi, upper=math.pi),
        )

    add_rotating_shaft("input_shaft", gear_steel, 0.024, gear_mesh)
    add_rotating_shaft("counter_shaft", gear_bronze, 0.026, counter_gear_mesh)
    add_rotating_shaft("output_shaft", gear_steel, 0.024, gear_mesh)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for shaft_name in ("input_shaft", "counter_shaft", "output_shaft"):
        joint = object_model.get_articulation(f"{shaft_name}_joint")
        ctx.check(
            f"{shaft_name} uses a revolute bearing axis",
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_within(
        "input_shaft",
        "frame",
        axes="xz",
        inner_elem="input_shaft_barrel",
        outer_elem="input_bearing_0",
        margin=0.0,
        name="input shaft runs through its bearing block",
    )
    ctx.expect_within(
        "counter_shaft",
        "frame",
        axes="xz",
        inner_elem="counter_shaft_barrel",
        outer_elem="counter_bearing_0",
        margin=0.0,
        name="countershaft runs through its bearing block",
    )
    ctx.expect_within(
        "output_shaft",
        "frame",
        axes="xz",
        inner_elem="output_shaft_barrel",
        outer_elem="output_bearing_0",
        margin=0.0,
        name="output shaft runs through its bearing block",
    )
    ctx.expect_overlap(
        "input_shaft",
        "counter_shaft",
        axes="x",
        elem_a="input_shaft_spur_gear",
        elem_b="counter_shaft_spur_gear",
        min_overlap=0.010,
        name="input gear reaches the counter gear plane",
    )
    ctx.expect_overlap(
        "output_shaft",
        "counter_shaft",
        axes="x",
        elem_a="output_shaft_spur_gear",
        elem_b="counter_shaft_spur_gear",
        min_overlap=0.010,
        name="output gear reaches the counter gear plane",
    )

    return ctx.report()


object_model = build_object_model()
