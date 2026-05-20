from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = model.material("walnut", rgba=(0.42, 0.28, 0.18, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.25, 0.27, 0.30, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.75, 0.77, 0.79, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.06, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.460, 0.360, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=walnut,
        name="plinth_body",
    )
    plinth.visual(
        Box((0.438, 0.338, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=satin_black,
        name="top_deck",
    )
    plinth.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=dark_metal,
        name="main_bearing_hub",
    )
    for index, (x, y) in enumerate(
        (
            (-0.180, -0.130),
            (-0.180, 0.130),
            (0.180, -0.130),
            (0.180, 0.130),
        )
    ):
        plinth.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber_black,
            name=f"foot_{index}",
        )

    plinth.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.165, 0.102, 0.054)),
        material=dark_metal,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.165, 0.102, 0.062)),
        material=brushed_aluminum,
        name="tonearm_bearing_collar",
    )
    plinth.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.160, -0.018, 0.053)),
        material=dark_metal,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.018, 0.008, 0.004)),
        origin=Origin(xyz=(0.160, -0.018, 0.063)),
        material=dark_metal,
        name="arm_rest_cradle",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.460, 0.360, 0.082)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.150, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brushed_aluminum,
        name="platter_rim",
    )
    platter.visual(
        Cylinder(radius=0.143, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=rubber_black,
        name="slipmat",
    )
    platter.visual(
        Cylinder(radius=0.002, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=brushed_aluminum,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.150, length=0.024),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_hub",
    )
    tonearm.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.000, 0.000),
                    (0.004, -0.060, 0.000),
                    (0.002, -0.120, -0.001),
                    (-0.006, -0.166, -0.003),
                ],
                radius=0.0042,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "tonearm_tube",
        ),
        material=brushed_aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.0032, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="rear_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.049, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.018, 0.028, 0.004)),
        origin=Origin(xyz=(-0.007, -0.175, -0.004), rpy=(-0.18, 0.0, -0.12)),
        material=brushed_aluminum,
        name="headshell",
    )
    tonearm.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(-0.007, -0.185, -0.010)),
        material=satin_black,
        name="cartridge_body",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.040, 0.250, 0.030)),
        mass=0.25,
        origin=Origin(xyz=(0.0, -0.065, -0.002)),
    )

    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.165, 0.102, 0.073)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    guard_frame = model.part("guard_frame")
    guard_frame.visual(
        mesh_from_geometry(TorusGeometry(radius=0.185, tube=0.005), "guard_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_metal,
        name="guard_ring",
    )
    support_angles = (math.radians(120.0), math.radians(240.0), math.radians(330.0))
    for index, angle in enumerate(support_angles):
        x = 0.185 * math.cos(angle)
        y = 0.185 * math.sin(angle)
        guard_frame.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(x, y, 0.046)),
            material=dark_metal,
            name=f"guard_foot_{index}",
        )
        guard_frame.visual(
            Cylinder(radius=0.0045, length=0.056),
            origin=Origin(xyz=(x, y, 0.076)),
            material=dark_metal,
            name=f"guard_post_{index}",
        )
    guard_frame.inertial = Inertial.from_geometry(
        Box((0.400, 0.400, 0.070)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    model.articulation(
        "plinth_to_guard_frame",
        ArticulationType.FIXED,
        parent=plinth,
        child=guard_frame,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    guard_frame = object_model.get_part("guard_frame")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_rim",
        negative_elem="top_deck",
        min_gap=0.0015,
        max_gap=0.0045,
        name="platter sits just above the top deck",
    )
    ctx.expect_within(
        platter,
        guard_frame,
        axes="xy",
        outer_elem="guard_ring",
        margin=0.040,
        name="guard ring surrounds the platter footprint",
    )
    ctx.expect_gap(
        guard_frame,
        platter,
        axis="z",
        positive_elem="guard_ring",
        negative_elem="platter_rim",
        min_gap=0.030,
        max_gap=0.060,
        name="guard ring remains above the rotating platter",
    )

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    rest_cartridge = ctx.part_element_world_aabb(tonearm, elem="cartridge_body")
    with ctx.pose({tonearm_joint: 1.05}):
        swung_cartridge = ctx.part_element_world_aabb(tonearm, elem="cartridge_body")

    rest_x = _aabb_center_x(rest_cartridge)
    swung_x = _aabb_center_x(swung_cartridge)
    ctx.check(
        "tonearm swings inward over the platter",
        rest_x is not None and swung_x is not None and swung_x < rest_x - 0.10,
        details=f"rest_x={rest_x}, swung_x={swung_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
