from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rpy_for_cylinder_axis(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (-math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
    *,
    axis: str = "z",
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_rpy_for_cylinder_axis(axis)),
        material=material,
        name=name,
    )


def _chamfered_box_mesh(
    name: str,
    size: tuple[float, float, float],
    *,
    chamfer: float = 0.003,
):
    sx, sy, sz = size
    body = cq.Workplane("XY").box(sx, sy, sz)
    if chamfer > 0.0:
        body = body.edges("|Z").chamfer(min(chamfer, sx * 0.10, sy * 0.10))
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.12)


def _add_chamfered_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
    *,
    chamfer: float = 0.003,
) -> None:
    part.visual(
        _chamfered_box_mesh(name, size, chamfer=chamfer),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rpr_mechanical_study")

    aluminum = model.material("bead_blasted_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    ground_steel = model.material("ground_guide_steel", rgba=(0.82, 0.83, 0.80, 1.0))
    cover = model.material("removable_access_cover", rgba=(0.22, 0.25, 0.27, 1.0))
    bronze = model.material("oil_bronze_bushing", rgba=(0.68, 0.45, 0.20, 1.0))
    red = model.material("red_stop_marker", rgba=(0.75, 0.12, 0.08, 1.0))

    # Root: low, fabricated test-stand base with a vertical rotary bearing stack.
    base = model.part("base_frame")
    _add_chamfered_box(base, "floor_plate", (0.76, 0.46, 0.035), (0.0, 0.0, 0.0175), dark_steel, chamfer=0.008)
    _add_chamfered_box(base, "left_foot_bar", (0.72, 0.055, 0.025), (0.0, -0.185, 0.0475), dark_steel, chamfer=0.004)
    _add_chamfered_box(base, "right_foot_bar", (0.72, 0.055, 0.025), (0.0, 0.185, 0.0475), dark_steel, chamfer=0.004)
    _add_cylinder(base, "pedestal_column", 0.110, 0.070, (0.0, 0.0, 0.070), aluminum)
    _add_cylinder(base, "lower_bearing_race", 0.165, 0.025, (0.0, 0.0, 0.1175), ground_steel)
    _add_cylinder(base, "spindle_stub", 0.050, 0.027, (0.0, 0.0, 0.1435), ground_steel)
    _add_box(base, "front_gusset", (0.050, 0.160, 0.065), (0.115, 0.0, 0.0675), aluminum)
    _add_box(base, "rear_gusset", (0.050, 0.160, 0.065), (-0.115, 0.0, 0.0675), aluminum)
    _add_chamfered_box(base, "bearing_access_cover", (0.145, 0.070, 0.006), (-0.245, 0.0, 0.038), cover, chamfer=0.002)
    for i, (x, y) in enumerate(
        (
            (-0.300, -0.155),
            (0.300, -0.155),
            (-0.300, 0.155),
            (0.300, 0.155),
        )
    ):
        _add_cylinder(base, f"anchor_bolt_{i}", 0.015, 0.008, (x, y, 0.0435), ground_steel)
    for i, y in enumerate((-0.025, 0.025)):
        _add_cylinder(base, f"cover_screw_{i}", 0.006, 0.004, (-0.245, y, 0.043), ground_steel)

    # First moving module: a discrete yaw table carrying the fixed prismatic guide.
    rotary = model.part("rotary_module")
    _add_cylinder(rotary, "turntable_disc", 0.145, 0.032, (0.0, 0.0, 0.016), ground_steel)
    _add_cylinder(rotary, "upper_bearing_race", 0.120, 0.018, (0.0, 0.0, 0.041), aluminum)
    _add_chamfered_box(rotary, "riser_block", (0.210, 0.165, 0.082), (0.055, 0.0, 0.091), aluminum, chamfer=0.006)
    _add_chamfered_box(rotary, "rail_bed", (0.870, 0.205, 0.045), (0.395, 0.0, 0.1545), aluminum, chamfer=0.006)
    _add_chamfered_box(rotary, "near_end_support", (0.060, 0.255, 0.100), (0.035, 0.0, 0.215), aluminum, chamfer=0.004)
    _add_chamfered_box(rotary, "far_end_support", (0.060, 0.255, 0.100), (0.755, 0.0, 0.215), aluminum, chamfer=0.004)
    for ix, x in enumerate((0.18, 0.395, 0.61)):
        for iy, y in enumerate((-0.080, 0.080)):
            _add_box(rotary, f"rail_saddle_{ix}_{iy}", (0.054, 0.046, 0.020), (x, y, 0.187), dark_steel)
    for i, y in enumerate((-0.080, 0.080)):
        _add_cylinder(rotary, f"guide_rail_{i}", 0.0175, 0.800, (0.395, y, 0.218), ground_steel, axis="x")
    _add_chamfered_box(rotary, "slide_access_cover", (0.235, 0.095, 0.007), (0.395, 0.0, 0.1805), cover, chamfer=0.002)
    for i, x in enumerate((0.305, 0.485)):
        _add_cylinder(rotary, f"slide_cover_screw_{i}", 0.005, 0.0035, (x, 0.0, 0.18575), ground_steel)
    _add_box(rotary, "near_travel_stop", (0.024, 0.185, 0.045), (0.120, 0.0, 0.236), red)
    _add_box(rotary, "far_travel_stop", (0.024, 0.185, 0.045), (0.675, 0.0, 0.236), red)

    yaw = model.articulation(
        "base_to_rotary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary,
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.4, lower=-1.20, upper=1.20),
    )

    # Linear stage: saddle and carriage bridge riding on the two guide rails.
    linear = model.part("linear_stage")
    _add_chamfered_box(linear, "saddle_plate", (0.220, 0.265, 0.032), (0.0, 0.0, 0.0675), aluminum, chamfer=0.005)
    for ix, x in enumerate((-0.068, 0.068)):
        for iy, y in enumerate((-0.080, 0.080)):
            _add_chamfered_box(
                linear,
                f"shoe_{ix}_{iy}",
                (0.080, 0.052, 0.034),
                (x, y, 0.0345),
                dark_steel,
                chamfer=0.002,
            )
            _add_cylinder(linear, f"shoe_cap_screw_{ix}_{iy}", 0.0055, 0.004, (x, y, 0.0855), ground_steel)
    _add_chamfered_box(linear, "top_access_cover", (0.150, 0.155, 0.007), (-0.025, 0.0, 0.0875), cover, chamfer=0.002)
    for i, (x, y) in enumerate(((-0.075, -0.055), (0.025, -0.055), (-0.075, 0.055), (0.025, 0.055))):
        _add_cylinder(linear, f"carriage_cover_screw_{i}", 0.0048, 0.0035, (x, y, 0.09275), ground_steel)
    _add_chamfered_box(linear, "clevis_base", (0.135, 0.205, 0.026), (0.145, 0.0, 0.0965), aluminum, chamfer=0.004)
    _add_chamfered_box(linear, "clevis_cheek_0", (0.080, 0.018, 0.125), (0.165, -0.075, 0.172), aluminum, chamfer=0.003)
    _add_chamfered_box(linear, "clevis_cheek_1", (0.080, 0.018, 0.125), (0.165, 0.075, 0.172), aluminum, chamfer=0.003)
    _add_chamfered_box(linear, "clevis_rear_bridge", (0.026, 0.170, 0.098), (0.116, 0.0, 0.1585), aluminum, chamfer=0.003)
    for i, y in enumerate((-0.089, 0.089)):
        _add_cylinder(linear, f"wrist_pin_cap_{i}", 0.024, 0.010, (0.165, y, 0.205), ground_steel, axis="y")
    _add_cylinder(linear, "wrist_bushing_0", 0.029, 0.012, (0.165, -0.066, 0.205), bronze, axis="y")
    _add_cylinder(linear, "wrist_bushing_1", 0.029, 0.012, (0.165, 0.066, 0.205), bronze, axis="y")
    _add_box(linear, "low_stop_boss", (0.032, 0.150, 0.020), (0.205, 0.0, 0.1195), red)
    _add_box(linear, "high_stop_stalk", (0.018, 0.022, 0.070), (0.205, 0.095, 0.245), red)
    _add_box(linear, "high_stop_boss", (0.034, 0.030, 0.020), (0.220, 0.100, 0.290), red)

    slide = model.articulation(
        "rotary_to_linear",
        ArticulationType.PRISMATIC,
        parent=rotary,
        child=linear,
        origin=Origin(xyz=(0.250, 0.0, 0.218)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.35, lower=0.0, upper=0.340),
    )

    # Distal wrist: compact hinged test link held between the carriage clevis cheeks.
    wrist = model.part("wrist_link")
    _add_cylinder(wrist, "hinge_boss", 0.030, 0.120, (0.0, 0.0, 0.0), bronze, axis="y")
    _add_chamfered_box(wrist, "hinge_lug", (0.095, 0.096, 0.074), (0.026, 0.0, 0.0), aluminum, chamfer=0.004)
    _add_chamfered_box(wrist, "wrist_arm", (0.315, 0.074, 0.046), (0.190, 0.0, 0.0), aluminum, chamfer=0.005)
    _add_chamfered_box(wrist, "arm_top_cover", (0.180, 0.052, 0.007), (0.195, 0.0, 0.0265), cover, chamfer=0.002)
    for i, x in enumerate((0.130, 0.260)):
        _add_cylinder(wrist, f"wrist_cover_screw_{i}", 0.0045, 0.003, (x, 0.0, 0.0315), ground_steel)
    _add_cylinder(wrist, "distal_bearing_eye", 0.043, 0.060, (0.370, 0.0, 0.0), ground_steel, axis="y")
    _add_cylinder(wrist, "distal_bore_face", 0.024, 0.064, (0.370, 0.0, 0.0), dark_steel, axis="y")
    _add_box(wrist, "wrist_stop_tab", (0.032, 0.125, 0.018), (0.070, 0.0, 0.032), red)

    pitch = model.articulation(
        "linear_to_wrist",
        ArticulationType.REVOLUTE,
        parent=linear,
        child=wrist,
        origin=Origin(xyz=(0.165, 0.0, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-0.65, upper=0.95),
    )

    # Metadata is only descriptive; joints above remain the authoritative mechanism.
    model.meta["study_notes"] = {
        "chain": "revolute-prismatic-revolute",
        "axes": {
            yaw.name: "vertical yaw axis through the bearing stack",
            slide.name: "linear travel along the exposed guide rails",
            pitch.name: "distal pitch wrist through a clevis-supported hinge",
        },
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    rotary = object_model.get_part("rotary_module")
    linear = object_model.get_part("linear_stage")
    wrist = object_model.get_part("wrist_link")
    yaw = object_model.get_articulation("base_to_rotary")
    slide = object_model.get_articulation("rotary_to_linear")
    pitch = object_model.get_articulation("linear_to_wrist")

    ctx.check(
        "explicit R-P-R joint sequence",
        [j.articulation_type for j in (yaw, slide, pitch)]
        == [ArticulationType.REVOLUTE, ArticulationType.PRISMATIC, ArticulationType.REVOLUTE],
        details=f"types={[j.articulation_type for j in (yaw, slide, pitch)]}",
    )
    ctx.check(
        "discrete orthogonal axes",
        yaw.axis == (0.0, 0.0, 1.0)
        and slide.axis == (1.0, 0.0, 0.0)
        and pitch.axis == (0.0, -1.0, 0.0),
        details=f"axes={yaw.axis}, {slide.axis}, {pitch.axis}",
    )

    ctx.expect_gap(
        rotary,
        base,
        axis="z",
        positive_elem="turntable_disc",
        negative_elem="spindle_stub",
        max_penetration=0.00001,
        max_gap=0.001,
        name="yaw table seats on spindle stub",
    )
    ctx.expect_gap(
        linear,
        rotary,
        axis="z",
        positive_elem="shoe_0_0",
        negative_elem="guide_rail_0",
        max_penetration=0.00001,
        max_gap=0.002,
        name="linear shoe rides on guide rail",
    )
    ctx.expect_overlap(
        linear,
        rotary,
        axes="x",
        elem_a="saddle_plate",
        elem_b="guide_rail_0",
        min_overlap=0.18,
        name="carriage saddle bridges rail length at rest",
    )
    ctx.expect_within(
        wrist,
        linear,
        axes="y",
        inner_elem="hinge_boss",
        outer_elem="clevis_rear_bridge",
        margin=0.0,
        name="wrist boss remains centered in clevis span",
    )

    rest_linear = ctx.part_world_position(linear)
    with ctx.pose({slide: 0.340}):
        extended_linear = ctx.part_world_position(linear)
        ctx.expect_overlap(
            linear,
            rotary,
            axes="x",
            elem_a="saddle_plate",
            elem_b="guide_rail_0",
            min_overlap=0.18,
            name="extended carriage remains on rails",
        )
    ctx.check(
        "prismatic stage translates outward",
        rest_linear is not None and extended_linear is not None and extended_linear[0] > rest_linear[0] + 0.30,
        details=f"rest={rest_linear}, extended={extended_linear}",
    )

    rest_wrist = ctx.part_world_aabb(wrist)
    with ctx.pose({pitch: 0.80}):
        raised_wrist = ctx.part_world_aabb(wrist)
    ctx.check(
        "wrist hinge pitches distal link upward",
        rest_wrist is not None and raised_wrist is not None and raised_wrist[1][2] > rest_wrist[1][2] + 0.18,
        details=f"rest={rest_wrist}, raised={raised_wrist}",
    )

    return ctx.report()


object_model = build_object_model()
