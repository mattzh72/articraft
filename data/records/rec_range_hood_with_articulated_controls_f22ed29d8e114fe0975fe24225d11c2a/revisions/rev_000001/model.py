import cadquery as cq
from sdk import (
    AllowedOverlap,
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


def build_canopy_cq():
    # Base lip
    lip = cq.Workplane("XY").rect(0.76, 0.50).extrude(0.05)
    # Tapering body
    taper = (
        cq.Workplane("XY")
        .workplane(offset=0.05)
        .rect(0.76, 0.50)
        .workplane(offset=0.20)
        .center(0, 0.125)
        .rect(0.30, 0.25)
        .loft()
    )
    canopy = lip.union(taper)

    # Hollow out the inside
    inner_lip = cq.Workplane("XY").workplane(offset=-0.01).rect(0.74, 0.48).extrude(0.06)
    inner_taper = (
        cq.Workplane("XY")
        .workplane(offset=0.05)
        .rect(0.74, 0.48)
        .workplane(offset=0.19)
        .center(0, 0.125)
        .rect(0.28, 0.23)
        .loft()
    )
    inner = inner_lip.union(inner_taper)
    canopy = canopy.cut(inner)

    # Add a front panel for the lights
    panel = (
        cq.Workplane("XY")
        .center(0, -0.1975)
        .rect(0.75, 0.095)
        .extrude(0.01)
    )
    canopy = canopy.union(panel)
    return canopy


def build_chimney_cq():
    chimney = cq.Workplane("XY").rect(0.30, 0.25).extrude(0.40)
    core = cq.Workplane("XY").rect(0.28, 0.23).extrude(0.40)
    return chimney.cut(core)


def build_filter_cq():
    # 0.35 wide, 0.38 long
    base = cq.Workplane("XY").rect(0.35, 0.38).extrude(0.002)
    for x in range(-15, 16):
        ridge = cq.Workplane("XY").center(x * 0.01, 0).rect(0.005, 0.36).extrude(0.008)
        base = base.union(ridge)
    return base


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="range_hood")

    brushed_metal = Material(
        name="brushed_metal",
        rgba=(0.7, 0.7, 0.72, 1.0),
    )
    black_plastic = Material(
        name="black_plastic",
        rgba=(0.1, 0.1, 0.1, 1.0),
    )
    light_glass = Material(
        name="light_glass",
        rgba=(1.0, 1.0, 0.9, 1.0),
    )

    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_cadquery(build_canopy_cq(), "canopy_mesh"),
        origin=Origin(),
        material=brushed_metal,
        name="canopy_shell",
    )

    chimney = model.part("chimney")
    chimney.visual(
        mesh_from_cadquery(build_chimney_cq(), "chimney_mesh"),
        origin=Origin(),
        material=brushed_metal,
        name="chimney_shell",
    )
    model.articulation(
        "canopy_to_chimney",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney,
        origin=Origin(xyz=(0.0, 0.125, 0.25)),
    )

    # Filters
    for side, sign in [("left", -1), ("right", 1)]:
        filter_part = model.part(f"filter_{side}")
        filter_part.visual(
            mesh_from_cadquery(build_filter_cq(), f"filter_mesh_{side}"),
            origin=Origin(xyz=(0.0, -0.19, -0.005)),
            material=brushed_metal,
            name=f"filter_shell_{side}",
        )
        model.articulation(
            f"canopy_to_filter_{side}",
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=filter_part,
            origin=Origin(xyz=(sign * 0.18, 0.24, 0.01)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=1.57, effort=1.0, velocity=1.0),
        )

    # Lights
    for side, sign in [("left", -1), ("right", 1)]:
        light = model.part(f"light_{side}")
        light.visual(
            Cylinder(radius=0.03, length=0.005),
            origin=Origin(xyz=(0, 0, -0.0025)),
            material=light_glass,
            name=f"light_lens_{side}",
        )
        model.articulation(
            f"canopy_to_light_{side}",
            ArticulationType.FIXED,
            parent=canopy,
            child=light,
            origin=Origin(xyz=(sign * 0.20, -0.20, 0.0)),
        )

    # Controls
    power_btn = model.part("power_button")
    power_btn.visual(
        Box((0.02, 0.005, 0.01)),
        origin=Origin(xyz=(0, -0.0025, 0)),
        material=black_plastic,
        name="power_btn_cap",
    )
    model.articulation(
        "canopy_to_power_btn",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=power_btn,
        origin=Origin(xyz=(-0.10, -0.25, 0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.003, effort=1.0, velocity=1.0),
    )

    light_btn = model.part("light_button")
    light_btn.visual(
        Box((0.02, 0.005, 0.01)),
        origin=Origin(xyz=(0, -0.0025, 0)),
        material=black_plastic,
        name="light_btn_cap",
    )
    model.articulation(
        "canopy_to_light_btn",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=light_btn,
        origin=Origin(xyz=(-0.05, -0.25, 0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.003, effort=1.0, velocity=1.0),
    )

    fan_knob = model.part("fan_knob")
    fan_knob.visual(
        Cylinder(radius=0.015, length=0.015),
        origin=Origin(xyz=(0, -0.0075, 0), rpy=(1.5708, 0, 0)),
        material=brushed_metal,
        name="fan_knob_cap",
    )
    fan_knob.visual(
        Box((0.002, 0.016, 0.005)),
        origin=Origin(xyz=(0, -0.015, 0.01)),
        material=black_plastic,
        name="fan_knob_indicator",
    )
    model.articulation(
        "canopy_to_fan_knob",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=fan_knob,
        origin=Origin(xyz=(0.10, -0.25, 0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.57, upper=1.57, effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    filter_left = object_model.get_part("filter_left")
    filter_right = object_model.get_part("filter_right")
    power_btn = object_model.get_part("power_button")
    light_btn = object_model.get_part("light_button")
    fan_knob = object_model.get_part("fan_knob")
    chimney = object_model.get_part("chimney")
    light_left = object_model.get_part("light_left")
    light_right = object_model.get_part("light_right")

    ctx.allow_overlap(canopy, chimney, reason="Chimney sits flush on canopy, slight precision overlap allowed.")
    ctx.allow_overlap(canopy, power_btn, reason="Button is embedded in the panel.")
    ctx.allow_overlap(canopy, light_btn, reason="Button is embedded in the panel.")
    ctx.allow_overlap(canopy, fan_knob, reason="Knob stem is embedded in the panel.")
    ctx.allow_overlap(canopy, light_left, reason="Light puck is surface mounted.")
    ctx.allow_overlap(canopy, light_right, reason="Light puck is surface mounted.")

    ctx.allow_overlap(canopy, filter_left, reason="Filter sits inside the canopy cavity.")
    ctx.allow_overlap(canopy, filter_right, reason="Filter sits inside the canopy cavity.")

    with ctx.pose(canopy_to_filter_left=1.0, canopy_to_filter_right=1.0):
        left_aabb = ctx.part_world_aabb(filter_left)
        right_aabb = ctx.part_world_aabb(filter_right)
        canopy_aabb = ctx.part_world_aabb(canopy)
        ctx.check(
            "filters_swing_down",
            left_aabb is not None and right_aabb is not None and canopy_aabb is not None
            and left_aabb[0][2] < canopy_aabb[0][2] - 0.1
            and right_aabb[0][2] < canopy_aabb[0][2] - 0.1,
            details="Filters should swing down below the canopy base.",
        )

    with ctx.pose(canopy_to_power_btn=0.003):
        ctx.expect_within(power_btn, canopy, axes="xz", margin=0.05)

    return ctx.report()


object_model = build_object_model()
