from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, center: tuple[float, float] = (0.0, 0.0), segments: int = 40):
    cx, cy = center
    return [
        (cx + radius * math.cos(2.0 * math.pi * i / segments), cy + radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _translated_profile(profile, dx: float, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _link_plate_mesh(length: float, name: str):
    outer = _translated_profile(
        rounded_rect_profile(length + 0.070, 0.066, 0.033, corner_segments=10),
        length / 2.0,
    )
    holes = [
        _circle_profile(0.0125, center=(0.0, 0.0), segments=44),
        _circle_profile(0.0125, center=(length, 0.0), segments=44),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, 0.014, center=True),
        name,
    )


def _annular_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=56),
            [_circle_profile(inner_radius, segments=48)],
            height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_equipment_bay_monitor_arm")

    powder_black = model.material("powder_black", rgba=(0.03, 0.035, 0.04, 1.0))
    satin_black = model.material("satin_black", rgba=(0.09, 0.095, 0.105, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    parkerized_pin = model.material("parkerized_pin", rgba=(0.35, 0.36, 0.36, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.005, 0.005, 0.006, 1.0))
    safety_blue = model.material("safety_blue_trim", rgba=(0.10, 0.22, 0.42, 1.0))

    rear_link_mesh = _link_plate_mesh(0.225, "rear_link_plate")
    front_link_mesh = _link_plate_mesh(0.205, "front_link_plate")
    pivot_collar_mesh = _annular_mesh(0.024, 0.0125, 0.006, "pivot_collar")
    pan_collar_mesh = _annular_mesh(0.031, 0.0125, 0.012, "pan_collar")

    back_bracket = model.part("back_bracket")
    back_bracket.visual(
        Box((0.035, 0.250, 0.190)),
        origin=Origin(xyz=(-0.056, 0.0, 0.0)),
        material=powder_black,
        name="wall_plate",
    )
    back_bracket.visual(
        Box((0.100, 0.084, 0.010)),
        origin=Origin(xyz=(0.002, 0.0, 0.022)),
        material=dark_graphite,
        name="upper_clevis_leaf",
    )
    back_bracket.visual(
        Box((0.100, 0.084, 0.010)),
        origin=Origin(xyz=(0.002, 0.0, -0.022)),
        material=dark_graphite,
        name="lower_clevis_leaf",
    )
    back_bracket.visual(
        Cylinder(radius=0.0105, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=parkerized_pin,
        name="rear_pivot_pin",
    )
    for y in (-0.082, 0.082):
        for z in (-0.060, 0.060):
            back_bracket.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(-0.037, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=parkerized_pin,
                name=f"mount_screw_{y}_{z}",
            )

    rear_link = model.part("rear_link")
    rear_link.visual(rear_link_mesh, material=satin_black, name="link_plate")
    rear_link.visual(
        pivot_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_graphite,
        name="rear_collar",
    )
    rear_link.visual(
        pivot_collar_mesh,
        origin=Origin(xyz=(0.225, 0.0, 0.010)),
        material=dark_graphite,
        name="front_collar",
    )
    rear_link.visual(
        Cylinder(radius=0.0105, length=0.046),
        origin=Origin(xyz=(0.225, 0.0, 0.022)),
        material=parkerized_pin,
        name="front_pivot_pin",
    )
    rear_link.visual(
        Cylinder(radius=0.021, length=0.004),
        origin=Origin(xyz=(0.225, 0.0, -0.009)),
        material=parkerized_pin,
        name="front_pin_head",
    )
    rear_link.visual(
        Box((0.135, 0.010, 0.006)),
        origin=Origin(xyz=(0.1125, 0.0, 0.008)),
        material=safety_blue,
        name="low_profile_witness_line",
    )

    front_link = model.part("front_link")
    front_link.visual(
        front_link_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=satin_black,
        name="link_plate",
    )
    front_link.visual(
        pivot_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=dark_graphite,
        name="rear_collar",
    )
    front_link.visual(
        pivot_collar_mesh,
        origin=Origin(xyz=(0.205, 0.0, 0.034)),
        material=dark_graphite,
        name="front_collar",
    )
    front_link.visual(
        Cylinder(radius=0.0105, length=0.052),
        origin=Origin(xyz=(0.205, 0.0, 0.046)),
        material=parkerized_pin,
        name="head_pivot_pin",
    )
    front_link.visual(
        Cylinder(radius=0.021, length=0.004),
        origin=Origin(xyz=(0.205, 0.0, 0.016)),
        material=parkerized_pin,
        name="head_pin_head",
    )
    front_link.visual(
        Box((0.118, 0.010, 0.006)),
        origin=Origin(xyz=(0.103, 0.0, 0.032)),
        material=safety_blue,
        name="low_profile_witness_line",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        pan_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=dark_graphite,
        name="pan_bearing_ring",
    )
    head_swivel.visual(
        Box((0.080, 0.026, 0.018)),
        origin=Origin(xyz=(0.054, 0.0, 0.058)),
        material=dark_graphite,
        name="short_neck",
    )
    head_swivel.visual(
        Box((0.030, 0.030, 0.030)),
        origin=Origin(xyz=(0.083, 0.0, 0.045)),
        material=dark_graphite,
        name="neck_drop",
    )
    head_swivel.visual(
        Box((0.040, 0.238, 0.020)),
        origin=Origin(xyz=(0.092, 0.0, 0.033)),
        material=dark_graphite,
        name="yoke_bridge",
    )
    head_swivel.visual(
        Box((0.034, 0.012, 0.096)),
        origin=Origin(xyz=(0.102, 0.106, 0.070)),
        material=dark_graphite,
        name="yoke_cheek_0",
    )
    head_swivel.visual(
        Box((0.034, 0.012, 0.096)),
        origin=Origin(xyz=(0.102, -0.106, 0.070)),
        material=dark_graphite,
        name="yoke_cheek_1",
    )
    for y, name in ((0.106, "cheek_bore_0"), (-0.106, "cheek_bore_1")):
        head_swivel.visual(
            Cylinder(radius=0.015, length=0.003),
            origin=Origin(xyz=(0.086, y, 0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_black,
            name=name,
        )

    display_plate = model.part("display_plate")
    display_plate.visual(
        Cylinder(radius=0.012, length=0.232),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=parkerized_pin,
        name="tilt_trunnion",
    )
    display_plate.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="tilt_hub",
    )
    display_plate.visual(
        Box((0.020, 0.180, 0.132)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=satin_black,
        name="display_plate_shell",
    )
    display_plate.visual(
        Box((0.008, 0.108, 0.082)),
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        material=safety_blue,
        name="vesa_pad",
    )
    for y in (-0.040, 0.040):
        for z in (-0.030, 0.030):
            display_plate.visual(
                Cylinder(radius=0.006, length=0.003),
                origin=Origin(xyz=(0.053, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=rubber_black,
                name=f"vesa_recess_{y}_{z}",
            )

    model.articulation(
        "bracket_yaw",
        ArticulationType.REVOLUTE,
        parent=back_bracket,
        child=rear_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=rear_link,
        child=front_link,
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.4, lower=-1.85, upper=1.85),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=front_link,
        child=head_swivel,
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=-1.40, upper=1.40),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=display_plate,
        origin=Origin(xyz=(0.102, 0.0, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    back_bracket = object_model.get_part("back_bracket")
    rear_link = object_model.get_part("rear_link")
    front_link = object_model.get_part("front_link")
    head_swivel = object_model.get_part("head_swivel")
    display_plate = object_model.get_part("display_plate")

    bracket_yaw = object_model.get_articulation("bracket_yaw")
    elbow_yaw = object_model.get_articulation("elbow_yaw")
    head_pan = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "four user-facing revolute axes",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (bracket_yaw, elbow_yaw, head_pan, head_tilt)
        ),
        details="The two arm pivots, head swivel, and head tilt must all be revolute joints.",
    )

    ctx.allow_overlap(
        head_swivel,
        display_plate,
        elem_a="yoke_cheek_0",
        elem_b="tilt_trunnion",
        reason="The tilt trunnion is intentionally captured in the cheek bore proxy.",
    )
    ctx.allow_overlap(
        head_swivel,
        display_plate,
        elem_a="yoke_cheek_1",
        elem_b="tilt_trunnion",
        reason="The tilt trunnion is intentionally captured in the opposite cheek bore proxy.",
    )
    ctx.allow_overlap(
        back_bracket,
        rear_link,
        elem_a="rear_pivot_pin",
        elem_b="link_plate",
        reason="The grounded bracket pin intentionally passes through the rear link pivot bore.",
    )
    ctx.allow_overlap(
        back_bracket,
        rear_link,
        elem_a="rear_pivot_pin",
        elem_b="rear_collar",
        reason="The grounded bracket pin is intentionally captured by the raised rear-link collar.",
    )
    ctx.allow_overlap(
        rear_link,
        front_link,
        elem_a="front_pivot_pin",
        elem_b="link_plate",
        reason="The elbow pin intentionally passes through the stacked front link pivot bore.",
    )
    ctx.allow_overlap(
        rear_link,
        front_link,
        elem_a="front_pivot_pin",
        elem_b="rear_collar",
        reason="The elbow pin is intentionally captured by the raised front-link collar.",
    )
    ctx.allow_overlap(
        front_link,
        head_swivel,
        elem_a="head_pivot_pin",
        elem_b="pan_bearing_ring",
        reason="The head swivel pin is intentionally captured inside the pan bearing ring proxy.",
    )
    ctx.expect_overlap(
        back_bracket,
        rear_link,
        axes="xy",
        elem_a="rear_pivot_pin",
        elem_b="link_plate",
        min_overlap=0.010,
        name="bracket pivot pin is centered in rear link bore",
    )
    ctx.expect_overlap(
        back_bracket,
        rear_link,
        axes="xy",
        elem_a="rear_pivot_pin",
        elem_b="rear_collar",
        min_overlap=0.010,
        name="bracket pivot pin passes through raised rear collar",
    )
    ctx.expect_overlap(
        rear_link,
        front_link,
        axes="xy",
        elem_a="front_pivot_pin",
        elem_b="link_plate",
        min_overlap=0.010,
        name="elbow pivot pin is centered in front link bore",
    )
    ctx.expect_overlap(
        rear_link,
        front_link,
        axes="xy",
        elem_a="front_pivot_pin",
        elem_b="rear_collar",
        min_overlap=0.010,
        name="elbow pivot pin passes through raised front collar",
    )
    ctx.expect_overlap(
        front_link,
        head_swivel,
        axes="xy",
        elem_a="head_pivot_pin",
        elem_b="pan_bearing_ring",
        min_overlap=0.010,
        name="head pan pin is centered in bearing ring",
    )
    ctx.expect_overlap(
        display_plate,
        head_swivel,
        axes="y",
        elem_a="tilt_trunnion",
        elem_b="yoke_cheek_0",
        min_overlap=0.006,
        name="trunnion reaches upper yoke cheek",
    )
    ctx.expect_overlap(
        display_plate,
        head_swivel,
        axes="y",
        elem_a="tilt_trunnion",
        elem_b="yoke_cheek_1",
        min_overlap=0.006,
        name="trunnion reaches lower yoke cheek",
    )
    ctx.expect_gap(
        rear_link,
        back_bracket,
        axis="z",
        min_gap=-0.001,
        max_gap=0.020,
        positive_elem="link_plate",
        negative_elem="lower_clevis_leaf",
        name="rear link sits in bracket clevis gap",
    )
    ctx.expect_gap(
        front_link,
        rear_link,
        axis="z",
        min_gap=0.002,
        max_gap=0.040,
        positive_elem="link_plate",
        negative_elem="link_plate",
        name="front link is stacked just above rear link",
    )
    ctx.expect_overlap(
        rear_link,
        front_link,
        axes="xy",
        elem_a="front_collar",
        elem_b="rear_collar",
        min_overlap=0.020,
        name="elbow collars share the same pivot footprint",
    )
    ctx.expect_overlap(
        front_link,
        head_swivel,
        axes="xy",
        elem_a="front_collar",
        elem_b="pan_bearing_ring",
        min_overlap=0.020,
        name="head pan bearing is seated over front link pivot",
    )

    rest_aabb = ctx.part_element_world_aabb(display_plate, elem="display_plate_shell")
    with ctx.pose({head_tilt: 0.45}):
        tilted_aabb = ctx.part_element_world_aabb(display_plate, elem="display_plate_shell")
    ctx.check(
        "tilt joint changes display plate pitch",
        rest_aabb is not None
        and tilted_aabb is not None
        and abs((tilted_aabb[1][2] - tilted_aabb[0][2]) - (rest_aabb[1][2] - rest_aabb[0][2])) > 0.003,
        details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
    )

    with ctx.pose({bracket_yaw: 0.45, elbow_yaw: -0.75, head_pan: 0.55}):
        ctx.expect_origin_distance(
            display_plate,
            back_bracket,
            axes="xy",
            min_dist=0.18,
            max_dist=0.55,
            name="folded arm keeps display within equipment bay reach",
        )

    return ctx.report()


object_model = build_object_model()
