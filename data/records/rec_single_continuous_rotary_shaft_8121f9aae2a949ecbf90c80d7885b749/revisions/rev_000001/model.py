from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_TOP_Z = 0.035
SHAFT_AXIS_Z = 0.120
BEARING_X_CENTERS = (-0.160, 0.160)
BEARING_WIDTH_X = 0.075


def _bearing_block_shape() -> cq.Workplane:
    """One low pillow-block bearing with a through bore along global X."""

    pedestal_height = 0.048
    pedestal = (
        cq.Workplane("XY")
        .box(BEARING_WIDTH_X, 0.165, pedestal_height)
        .translate((0.0, 0.0, FOOT_TOP_Z - 0.002 + pedestal_height * 0.5))
    )
    bearing_housing = (
        cq.Workplane("YZ")
        .cylinder(BEARING_WIDTH_X, 0.069)
        .translate((0.0, 0.0, SHAFT_AXIS_Z))
    )
    bore_cutter = (
        cq.Workplane("YZ")
        .cylinder(BEARING_WIDTH_X * 1.35, 0.052)
        .translate((0.0, 0.0, SHAFT_AXIS_Z))
    )
    block = pedestal.union(bearing_housing).cut(bore_cutter)
    return block.edges("|X").fillet(0.003)


def _bearing_bushing_shape() -> cq.Workplane:
    """Bronze liner ring seated in each bearing block."""

    outer = (
        cq.Workplane("YZ")
        .cylinder(BEARING_WIDTH_X + 0.010, 0.0525)
        .translate((0.0, 0.0, SHAFT_AXIS_Z))
    )
    inner = (
        cq.Workplane("YZ")
        .cylinder(BEARING_WIDTH_X + 0.018, 0.033)
        .translate((0.0, 0.0, SHAFT_AXIS_Z))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_rotary_spindle")

    cast_iron = model.material("cast_iron", rgba=(0.23, 0.27, 0.30, 1.0))
    dark_plate = model.material("dark_plate", rgba=(0.12, 0.14, 0.15, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.62, 0.64, 0.65, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    bronze = model.material("bronze_liner", rgba=(0.78, 0.55, 0.25, 1.0))
    black = model.material("blackened_hardware", rgba=(0.03, 0.03, 0.035, 1.0))

    bearing_mesh = mesh_from_cadquery(_bearing_block_shape(), "bearing_block")
    bushing_mesh = mesh_from_cadquery(_bearing_bushing_shape(), "bearing_bushing")

    foot = model.part("foot")
    foot.visual(
        Box((0.700, 0.280, FOOT_TOP_Z)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_TOP_Z * 0.5)),
        material=dark_plate,
        name="base_plate",
    )
    foot.visual(
        Box((0.500, 0.105, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_TOP_Z + 0.006)),
        material=cast_iron,
        name="raised_land",
    )
    for x, block_name, bushing_name in (
        (BEARING_X_CENTERS[0], "bearing_block_0", "bearing_bushing_0"),
        (BEARING_X_CENTERS[1], "bearing_block_1", "bearing_bushing_1"),
    ):
        foot.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=cast_iron,
            name=block_name,
        )
        foot.visual(
            bushing_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=bronze,
            name=bushing_name,
        )

    for i, (x, y) in enumerate(
        ((-0.290, -0.092), (-0.290, 0.092), (0.290, -0.092), (0.290, 0.092))
    ):
        foot.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, y, FOOT_TOP_Z + 0.0015)),
            material=machined_steel,
            name=f"mount_washer_{i}",
        )
        foot.visual(
            Cylinder(radius=0.009, length=0.007),
            origin=Origin(xyz=(x, y, FOOT_TOP_Z + 0.0045)),
            material=black,
            name=f"mount_bolt_{i}",
        )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.034, length=0.560),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="shaft_body",
    )
    shaft.visual(
        Cylinder(radius=0.074, length=0.040),
        origin=Origin(xyz=(-0.300, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="end_flange",
    )
    shaft.visual(
        Cylinder(radius=0.044, length=0.030),
        origin=Origin(xyz=(-0.266, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="flange_boss",
    )
    for i, (y, z) in enumerate(((0.046, 0.030), (-0.046, 0.030), (0.046, -0.030), (-0.046, -0.030))):
        shaft.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(xyz=(-0.323, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"flange_bolt_{i}",
        )

    shaft.visual(
        Cylinder(radius=0.056, length=0.090),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="clamp_hub",
    )
    shaft.visual(
        Box((0.078, 0.026, 0.048)),
        origin=Origin(xyz=(0.040, 0.060, 0.0)),
        material=machined_steel,
        name="clamp_lug",
    )
    shaft.visual(
        Box((0.094, 0.004, 0.050)),
        origin=Origin(xyz=(0.040, 0.056, 0.0)),
        material=black,
        name="clamp_split",
    )
    shaft.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.040, 0.071, 0.017), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="clamp_screw",
    )
    shaft.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.040, 0.071, -0.017), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="clamp_pin",
    )

    foot.inertial = Inertial.from_geometry(
        Box((0.700, 0.280, 0.190)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.560),
        mass=2.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "shaft_rotation",
        ArticulationType.CONTINUOUS,
        parent=foot,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    foot = object_model.get_part("foot")
    shaft = object_model.get_part("shaft")
    joint = object_model.get_articulation("shaft_rotation")

    ctx.check(
        "single continuous shaft joint",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    for i, (block_name, bushing_name) in enumerate(
        (("bearing_block_0", "bearing_bushing_0"), ("bearing_block_1", "bearing_bushing_1"))
    ):
        ctx.allow_overlap(
            foot,
            shaft,
            elem_a=bushing_name,
            elem_b="shaft_body",
            reason="The shaft is intentionally captured in a simplified bronze bearing liner with tiny local interference.",
        )
        ctx.expect_overlap(
            shaft,
            foot,
            axes="x",
            elem_a="shaft_body",
            elem_b=block_name,
            min_overlap=0.060,
            name=f"shaft passes through bearing block {i}",
        )
        ctx.expect_within(
            shaft,
            foot,
            axes="yz",
            inner_elem="shaft_body",
            outer_elem=bushing_name,
            margin=0.001,
            name=f"shaft centered in bearing bushing {i}",
        )

    with ctx.pose({joint: math.pi / 2.0}):
        ctx.expect_overlap(
            shaft,
            foot,
            axes="x",
            elem_a="shaft_body",
            elem_b="bearing_block_0",
            min_overlap=0.060,
            name="rotated shaft remains captured",
        )

    return ctx.report()


object_model = build_object_model()
