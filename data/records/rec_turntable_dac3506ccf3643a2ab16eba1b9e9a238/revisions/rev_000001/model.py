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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = model.material("walnut", rgba=(0.37, 0.22, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.63, 0.65, 0.68, 1.0))
    felt = model.material("felt", rgba=(0.16, 0.18, 0.16, 1.0))
    chrome = model.material("chrome", rgba=(0.86, 0.87, 0.89, 1.0))

    plinth_size = (0.46, 0.355, 0.048)
    feet_height = 0.018
    body_top_z = feet_height + plinth_size[2]

    platter_axis = (-0.065, 0.0, body_top_z + 0.002)
    tonearm_pivot_xy = (0.145, 0.110)
    tonearm_joint_z = body_top_z + 0.022
    tonearm_rest_yaw = math.atan2(-0.012 - tonearm_pivot_xy[1], -0.080 - tonearm_pivot_xy[0])

    arm_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.010, 0.0, 0.022),
                (0.085, 0.0, 0.022),
                (0.185, 0.0, 0.019),
                (0.236, 0.0, 0.015),
            ],
            radius=0.0046,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
        "tonearm_tube",
    )

    plinth = model.part("plinth")
    plinth.visual(
        Box(plinth_size),
        origin=Origin(xyz=(0.0, 0.0, feet_height + plinth_size[2] / 2.0)),
        material=walnut,
        name="plinth_body",
    )
    plinth.visual(
        Box((0.445, 0.340, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, body_top_z - 0.0015)),
        material=satin_black,
        name="top_plate",
    )
    for name, x_sign, y_sign in (
        ("foot_front_left", -1.0, 1.0),
        ("foot_front_right", 1.0, 1.0),
        ("foot_rear_left", -1.0, -1.0),
        ("foot_rear_right", 1.0, -1.0),
    ):
        plinth.visual(
            Cylinder(radius=0.019, length=feet_height),
            origin=Origin(xyz=(x_sign * 0.175, y_sign * 0.125, feet_height / 2.0)),
            material=dark_rubber,
            name=name,
        )
    plinth.visual(
        Cylinder(radius=0.022, length=0.001),
        origin=Origin(xyz=(platter_axis[0], platter_axis[1], body_top_z + 0.0005)),
        material=brushed_metal,
        name="bearing_collar",
    )
    plinth.visual(
        Cylinder(radius=0.034, length=0.022),
        origin=Origin(xyz=(tonearm_pivot_xy[0], tonearm_pivot_xy[1], body_top_z + 0.011)),
        material=satin_black,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(
            xyz=(tonearm_pivot_xy[0] + 0.032, tonearm_pivot_xy[1] + 0.026, body_top_z + 0.013)
        ),
        material=chrome,
        name="armrest_post",
    )
    plinth.visual(
        Box((0.016, 0.006, 0.004)),
        origin=Origin(
            xyz=(tonearm_pivot_xy[0] + 0.032, tonearm_pivot_xy[1] + 0.026, body_top_z + 0.028)
        ),
        material=chrome,
        name="armrest_cradle",
    )
    plinth.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(-0.185, -0.125, body_top_z + 0.005)),
        material=brushed_metal,
        name="speed_selector",
    )
    plinth.visual(
        Box((0.030, 0.010, 0.003)),
        origin=Origin(xyz=(0.185, -0.140, body_top_z + 0.0015)),
        material=chrome,
        name="power_switch",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.46, 0.355, 0.084)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.147, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.142, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=felt,
        name="slipmat",
    )
    platter.visual(
        Cylinder(radius=0.0018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=chrome,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.147, length=0.020),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_black,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=brushed_metal,
        name="pivot_turret",
    )
    tonearm.visual(
        arm_tube_mesh,
        material=chrome,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.0035, length=0.048),
        origin=Origin(xyz=(-0.022, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(-0.058, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.032, 0.018, 0.004)),
        origin=Origin(xyz=(0.252, 0.0, 0.014)),
        material=satin_black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(0.255, 0.0, 0.006)),
        material=satin_black,
        name="cartridge",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.340, 0.060, 0.050)),
        mass=0.42,
        origin=Origin(xyz=(0.100, 0.0, 0.018)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=platter_axis),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(
            xyz=(tonearm_pivot_xy[0], tonearm_pivot_xy[1], tonearm_joint_z),
            rpy=(0.0, 0.0, tonearm_rest_yaw),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=-0.35,
            upper=0.38,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    arm_joint = object_model.get_articulation("tonearm_swing")

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="top_plate",
        min_gap=0.0,
        max_gap=0.010,
        name="platter sits just above the top plate",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        min_overlap=0.20,
        name="platter remains supported within the plinth footprint",
    )

    plinth_pos = ctx.part_world_position(plinth)
    platter_pos = ctx.part_world_position(platter)
    tonearm_pos = ctx.part_world_position(tonearm)
    ctx.check(
        "platter axis is offset from the plinth center",
        plinth_pos is not None
        and platter_pos is not None
        and abs(platter_pos[0] - plinth_pos[0]) > 0.04,
        details=f"plinth={plinth_pos}, platter={platter_pos}",
    )
    ctx.check(
        "tonearm pivot sits to the side of the platter",
        platter_pos is not None
        and tonearm_pos is not None
        and tonearm_pos[0] > platter_pos[0] + 0.16,
        details=f"platter={platter_pos}, tonearm={tonearm_pos}",
    )

    rest_headshell = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({arm_joint: 0.30}):
        swung_headshell = ctx.part_element_world_aabb(tonearm, elem="headshell")

    def _center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    rest_center = _center(rest_headshell)
    swung_center = _center(swung_headshell)
    ctx.check(
        "tonearm swings across the record area",
        rest_center is not None
        and swung_center is not None
        and swung_center[1] < rest_center[1] - 0.04,
        details=f"rest={rest_center}, swung={swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
