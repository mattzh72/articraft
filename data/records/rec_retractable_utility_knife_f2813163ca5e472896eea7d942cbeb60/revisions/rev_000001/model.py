from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_box_cutter")

    body_plastic = model.material("body_plastic", rgba=(0.18, 0.18, 0.20, 1.0))
    door_plastic = model.material("door_plastic", rgba=(0.24, 0.24, 0.27, 1.0))
    slider_orange = model.material("slider_orange", rgba=(0.86, 0.42, 0.08, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.56, 0.58, 0.62, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.130, 0.028, 0.0024)),
        origin=Origin(xyz=(0.000, 0.000, 0.0012)),
        material=body_plastic,
        name="bottom_shell",
    )
    body.visual(
        Box((0.123, 0.0025, 0.0120)),
        origin=Origin(xyz=(-0.0015, -0.01275, 0.0080)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((0.077, 0.0025, 0.0120)),
        origin=Origin(xyz=(-0.0235, 0.01275, 0.0080)),
        material=body_plastic,
        name="right_rear_wall",
    )
    body.visual(
        Box((0.010, 0.0025, 0.0120)),
        origin=Origin(xyz=(0.0550, 0.01275, 0.0080)),
        material=body_plastic,
        name="right_front_post",
    )
    body.visual(
        Box((0.032, 0.0025, 0.0044)),
        origin=Origin(xyz=(0.0340, 0.01275, 0.0022)),
        material=body_plastic,
        name="door_lower_sill",
    )
    body.visual(
        Box((0.032, 0.0025, 0.0032)),
        origin=Origin(xyz=(0.0340, 0.01275, 0.0138)),
        material=body_plastic,
        name="door_upper_sill",
    )
    body.visual(
        Box((0.080, 0.0070, 0.0018)),
        origin=Origin(xyz=(-0.0250, -0.0080, 0.0148)),
        material=body_plastic,
        name="left_top_rail",
    )
    body.visual(
        Box((0.077, 0.0070, 0.0018)),
        origin=Origin(xyz=(-0.0235, 0.0080, 0.0148)),
        material=body_plastic,
        name="right_top_rail",
    )
    body.visual(
        Box((0.024, 0.028, 0.0018)),
        origin=Origin(xyz=(0.0510, 0.0000, 0.0151)),
        material=body_plastic,
        name="nose_top_bridge",
    )
    body.visual(
        Box((0.006, 0.028, 0.0140)),
        origin=Origin(xyz=(-0.0620, 0.0000, 0.0070)),
        material=body_plastic,
        name="rear_cap",
    )
    body.visual(
        Box((0.004, 0.028, 0.0025)),
        origin=Origin(xyz=(0.0630, 0.0000, 0.00125)),
        material=body_plastic,
        name="nose_lower_lip",
    )
    body.visual(
        Box((0.004, 0.028, 0.0025)),
        origin=Origin(xyz=(0.0630, 0.0000, 0.01375)),
        material=body_plastic,
        name="nose_upper_lip",
    )
    body.visual(
        Cylinder(radius=0.0014, length=0.0026),
        origin=Origin(xyz=(0.0500, 0.0154, 0.0053)),
        material=dark_steel,
        name="hinge_knuckle_lower",
    )
    body.visual(
        Cylinder(radius=0.0014, length=0.0026),
        origin=Origin(xyz=(0.0500, 0.0154, 0.0117)),
        material=dark_steel,
        name="hinge_knuckle_upper",
    )

    carriage = model.part("blade_carriage")
    carriage.visual(
        Box((0.042, 0.014, 0.0040)),
        origin=Origin(),
        material=dark_steel,
        name="slider_block",
    )
    carriage.visual(
        Box((0.018, 0.010, 0.0026)),
        origin=Origin(xyz=(0.0300, 0.0000, -0.0002)),
        material=dark_steel,
        name="blade_clamp",
    )
    carriage.visual(
        Box((0.0060, 0.0034, 0.0044)),
        origin=Origin(xyz=(0.0030, 0.0000, 0.0042)),
        material=slider_orange,
        name="slider_stem",
    )
    carriage.visual(
        Box((0.011, 0.007, 0.0032)),
        origin=Origin(xyz=(0.0030, 0.0000, 0.0076)),
        material=slider_orange,
        name="thumb_button",
    )

    blade_profile = [
        (0.000, -0.0036),
        (0.018, -0.0036),
        (0.026, -0.0018),
        (0.034, 0.0000),
        (0.026, 0.0018),
        (0.018, 0.0036),
        (0.000, 0.0036),
    ]
    blade_mesh = mesh_from_geometry(
        ExtrudeGeometry(blade_profile, 0.0012, cap=True, center=True),
        "box_cutter_blade",
    )
    carriage.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0360, 0.0000, 0.0000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blade_steel,
        name="blade_edge",
    )

    door = model.part("service_door")
    door.visual(
        Box((0.032, 0.0020, 0.0100)),
        origin=Origin(xyz=(-0.0160, 0.0010, 0.0000)),
        material=door_plastic,
        name="door_panel",
    )
    door.visual(
        Box((0.006, 0.0016, 0.0040)),
        origin=Origin(xyz=(-0.0260, 0.0028, 0.0000)),
        material=slider_orange,
        name="door_latch_rib",
    )
    door.visual(
        Cylinder(radius=0.0014, length=0.0034),
        origin=Origin(xyz=(0.0000, 0.0014, 0.0000)),
        material=dark_steel,
        name="door_hinge_knuckle",
    )

    model.articulation(
        "body_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(-0.0060, 0.0000, 0.0060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.30,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0500, 0.0140, 0.0085)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
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

    body = object_model.get_part("body")
    carriage = object_model.get_part("blade_carriage")
    door = object_model.get_part("service_door")
    slide = object_model.get_articulation("body_to_blade_carriage")
    door_hinge = object_model.get_articulation("body_to_service_door")

    slide_upper = 0.018
    door_open = 1.10

    ctx.expect_within(
        carriage,
        body,
        axes="yz",
        inner_elem="slider_block",
        margin=0.0005,
        name="carriage block stays guided inside the handle at rest",
    )

    with ctx.pose({slide: slide_upper}):
        ctx.expect_within(
            carriage,
            body,
            axes="yz",
            inner_elem="slider_block",
            margin=0.0005,
            name="carriage block stays guided inside the handle when extended",
        )

    body_aabb = ctx.part_world_aabb(body)
    blade_rest = ctx.part_element_world_aabb(carriage, elem="blade_edge")
    lower_sill = ctx.part_element_world_aabb(body, elem="door_lower_sill")
    door_closed = ctx.part_element_world_aabb(door, elem="door_panel")

    with ctx.pose({slide: slide_upper}):
        blade_extended = ctx.part_element_world_aabb(carriage, elem="blade_edge")

    with ctx.pose({door_hinge: door_open}):
        door_opened = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "blade remains tucked behind the nose at rest",
        body_aabb is not None
        and blade_rest is not None
        and blade_rest[1][0] <= body_aabb[1][0] + 0.001,
        details=f"body_aabb={body_aabb}, blade_rest={blade_rest}",
    )
    ctx.check(
        "blade projects forward when extended",
        body_aabb is not None
        and blade_extended is not None
        and blade_extended[1][0] >= body_aabb[1][0] + 0.010,
        details=f"body_aabb={body_aabb}, blade_extended={blade_extended}",
    )
    ctx.check(
        "blade carriage slides toward the nose",
        blade_rest is not None
        and blade_extended is not None
        and blade_extended[1][0] > blade_rest[1][0] + 0.012,
        details=f"blade_rest={blade_rest}, blade_extended={blade_extended}",
    )
    ctx.check(
        "service door sits flush with the side frame when closed",
        lower_sill is not None
        and door_closed is not None
        and abs(door_closed[0][1] - lower_sill[1][1]) <= 0.0008,
        details=f"lower_sill={lower_sill}, door_closed={door_closed}",
    )
    ctx.check(
        "service door swings outward from the body side",
        door_closed is not None
        and door_opened is not None
        and ((door_opened[0][1] + door_opened[1][1]) * 0.5)
        > ((door_closed[0][1] + door_closed[1][1]) * 0.5) + 0.008,
        details=f"door_closed={door_closed}, door_opened={door_opened}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
