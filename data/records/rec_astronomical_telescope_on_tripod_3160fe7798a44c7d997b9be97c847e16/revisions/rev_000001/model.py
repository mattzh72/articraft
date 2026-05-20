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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_field_bino_refractor_mount")

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.30, 0.33, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    off_white = model.material("off_white", rgba=(0.88, 0.89, 0.87, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    ground_pier = model.part("ground_pier")
    ground_pier.visual(
        Cylinder(radius=0.23, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=graphite,
        name="ground_plate",
    )
    ground_pier.visual(
        Cylinder(radius=0.065, length=0.84),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=gunmetal,
        name="pier_column",
    )
    ground_pier.visual(
        Cylinder(radius=0.095, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        material=graphite,
        name="bearing_housing",
    )
    ground_pier.visual(
        Cylinder(radius=0.12, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 1.045)),
        material=gunmetal,
        name="bearing_flange",
    )
    brace_origin = Origin(xyz=(0.14, 0.0, 0.12), rpy=(0.0, -0.55, 0.0))
    ground_pier.visual(
        Box((0.28, 0.04, 0.10)),
        origin=brace_origin,
        material=gunmetal,
        name="brace_0",
    )
    ground_pier.visual(
        Box((0.28, 0.04, 0.10)),
        origin=Origin(
            xyz=(0.14 * math.cos(2.0 * math.pi / 3.0), 0.14 * math.sin(2.0 * math.pi / 3.0), 0.12),
            rpy=(0.0, -0.55, 2.0 * math.pi / 3.0),
        ),
        material=gunmetal,
        name="brace_1",
    )
    ground_pier.visual(
        Box((0.28, 0.04, 0.10)),
        origin=Origin(
            xyz=(0.14 * math.cos(4.0 * math.pi / 3.0), 0.14 * math.sin(4.0 * math.pi / 3.0), 0.12),
            rpy=(0.0, -0.55, 4.0 * math.pi / 3.0),
        ),
        material=gunmetal,
        name="brace_2",
    )
    ground_pier.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=1.06),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
    )

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.105, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=graphite,
        name="azimuth_drum",
    )
    azimuth_head.visual(
        Cylinder(radius=0.14, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=gunmetal,
        name="rotating_deck",
    )
    azimuth_head.visual(
        Box((0.10, 0.18, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=gunmetal,
        name="head_mast",
    )
    azimuth_head.visual(
        Box((0.02, 0.14, 0.10)),
        origin=Origin(xyz=(0.05, 0.0, 0.56)),
        material=graphite,
        name="shoulder_block",
    )
    azimuth_head.visual(
        Cylinder(radius=0.028, length=0.16),
        origin=Origin(xyz=(0.032, 0.0, 0.56), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="shoulder_axle_sleeve",
    )
    azimuth_head.visual(
        Box((0.10, 0.02, 0.18)),
        origin=Origin(xyz=(0.015, 0.05, 0.47)),
        material=graphite,
        name="left_cheek",
    )
    azimuth_head.visual(
        Box((0.10, 0.02, 0.18)),
        origin=Origin(xyz=(0.015, -0.05, 0.47)),
        material=graphite,
        name="right_cheek",
    )
    azimuth_head.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.61)),
        mass=7.0,
        origin=Origin(xyz=(0.02, 0.0, 0.305)),
    )

    parallelogram_arm = model.part("parallelogram_arm")
    parallelogram_arm.visual(
        Box((0.73, 0.03, 0.025)),
        origin=Origin(xyz=(0.385, 0.0, 0.07)),
        material=aluminum,
        name="upper_arm_bar",
    )
    parallelogram_arm.visual(
        Box((0.73, 0.03, 0.025)),
        origin=Origin(xyz=(0.385, 0.0, -0.07)),
        material=aluminum,
        name="lower_arm_bar",
    )
    parallelogram_arm.visual(
        Box((0.055, 0.07, 0.175)),
        origin=Origin(xyz=(0.0275, 0.0, 0.0)),
        material=gunmetal,
        name="rear_pivot_block",
    )
    parallelogram_arm.visual(
        Cylinder(radius=0.022, length=0.078),
        origin=Origin(xyz=(0.0275, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="rear_pivot_cap",
    )
    parallelogram_arm.visual(
        Box((0.055, 0.07, 0.175)),
        origin=Origin(xyz=(0.7225, 0.0, 0.0)),
        material=gunmetal,
        name="front_pivot_block",
    )
    parallelogram_arm.visual(
        Cylinder(radius=0.022, length=0.078),
        origin=Origin(xyz=(0.7225, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_pivot_cap",
    )
    parallelogram_arm.inertial = Inertial.from_geometry(
        Box((0.76, 0.18, 0.18)),
        mass=6.5,
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
    )

    bridge_bar = model.part("bridge_bar")
    bridge_bar.visual(
        Box((0.03, 0.055, 0.18)),
        origin=Origin(xyz=(0.015, 0.0, 0.09)),
        material=graphite,
        name="bridge_hanger",
    )
    bridge_bar.visual(
        Cylinder(radius=0.014, length=0.08),
        origin=Origin(xyz=(0.014, 0.0, 0.09), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="bridge_pivot_cap",
    )
    bridge_bar.visual(
        Box((0.22, 0.32, 0.03)),
        origin=Origin(xyz=(0.13, 0.0, 0.195)),
        material=aluminum,
        name="bridge_crossbar",
    )
    bridge_bar.visual(
        Box((0.09, 0.05, 0.04)),
        origin=Origin(xyz=(0.10, 0.13, 0.23)),
        material=gunmetal,
        name="left_saddle",
    )
    bridge_bar.visual(
        Box((0.09, 0.05, 0.04)),
        origin=Origin(xyz=(0.10, -0.13, 0.23)),
        material=gunmetal,
        name="right_saddle",
    )
    bridge_bar.inertial = Inertial.from_geometry(
        Box((0.24, 0.32, 0.25)),
        mass=3.5,
        origin=Origin(xyz=(0.12, 0.0, 0.125)),
    )

    def add_refractor(name: str) -> None:
        tube = model.part(name)
        tube.visual(
            Box((0.18, 0.05, 0.015)),
            origin=Origin(xyz=(0.0, 0.0, 0.0075)),
            material=graphite,
            name="mounting_rail",
        )
        tube.visual(
            Box((0.022, 0.05, 0.094)),
            origin=Origin(xyz=(-0.055, 0.0, 0.062)),
            material=gunmetal,
            name="rear_ring",
        )
        tube.visual(
            Box((0.022, 0.05, 0.094)),
            origin=Origin(xyz=(0.055, 0.0, 0.062)),
            material=gunmetal,
            name="front_ring",
        )
        tube.visual(
            Cylinder(radius=0.047, length=0.46),
            origin=Origin(xyz=(0.035, 0.0, 0.109), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name="optical_tube",
        )
        tube.visual(
            Cylinder(radius=0.055, length=0.036),
            origin=Origin(xyz=(0.238, 0.0, 0.109), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=off_white,
            name="objective_cell",
        )
        tube.visual(
            Cylinder(radius=0.058, length=0.16),
            origin=Origin(xyz=(0.328, 0.0, 0.109), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name="dew_shield",
        )
        tube.visual(
            Cylinder(radius=0.030, length=0.12),
            origin=Origin(xyz=(-0.24, 0.0, 0.109), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gunmetal,
            name="focuser_body",
        )
        tube.visual(
            Box((0.035, 0.04, 0.065)),
            origin=Origin(xyz=(-0.305, 0.0, 0.0675)),
            material=graphite,
            name="diagonal_body",
        )
        tube.visual(
            Cylinder(radius=0.018, length=0.09),
            origin=Origin(xyz=(-0.305, 0.0, 0.1325)),
            material=rubber,
            name="eyepiece",
        )
        tube.inertial = Inertial.from_geometry(
            Box((0.78, 0.12, 0.18)),
            mass=3.2,
            origin=Origin(xyz=(0.02, 0.0, 0.09)),
        )

    add_refractor("left_refractor")
    add_refractor("right_refractor")

    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=ground_pier,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2),
    )
    model.articulation(
        "shoulder_lift",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=parallelogram_arm,
        origin=Origin(xyz=(0.06, 0.0, 0.56)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.0, lower=-0.25, upper=1.05),
    )
    model.articulation(
        "bridge_leveling",
        ArticulationType.REVOLUTE,
        parent=parallelogram_arm,
        child=bridge_bar,
        origin=Origin(xyz=(0.75, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "bridge_to_left_refractor",
        ArticulationType.FIXED,
        parent=bridge_bar,
        child="left_refractor",
        origin=Origin(xyz=(0.10, 0.13, 0.25)),
    )
    model.articulation(
        "bridge_to_right_refractor",
        ArticulationType.FIXED,
        parent=bridge_bar,
        child="right_refractor",
        origin=Origin(xyz=(0.10, -0.13, 0.25)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ground_pier = object_model.get_part("ground_pier")
    azimuth_head = object_model.get_part("azimuth_head")
    parallelogram_arm = object_model.get_part("parallelogram_arm")
    bridge_bar = object_model.get_part("bridge_bar")
    left_refractor = object_model.get_part("left_refractor")
    right_refractor = object_model.get_part("right_refractor")

    azimuth_bearing = object_model.get_articulation("azimuth_bearing")
    shoulder_lift = object_model.get_articulation("shoulder_lift")
    bridge_leveling = object_model.get_articulation("bridge_leveling")

    ctx.expect_contact(
        azimuth_head,
        ground_pier,
        name="azimuth head sits on the bearing flange",
    )
    ctx.expect_contact(
        parallelogram_arm,
        azimuth_head,
        name="arm shoulder block contacts the head pivot",
    )
    ctx.expect_contact(
        bridge_bar,
        parallelogram_arm,
        name="bridge hanger meets the front pivot block",
    )
    ctx.expect_contact(
        left_refractor,
        bridge_bar,
        name="left refractor is mounted to the bridge saddle",
    )
    ctx.expect_contact(
        right_refractor,
        bridge_bar,
        name="right refractor is mounted to the bridge saddle",
    )
    ctx.expect_origin_distance(
        left_refractor,
        right_refractor,
        axes="y",
        min_dist=0.24,
        max_dist=0.28,
        name="twin refractors keep binocular spacing",
    )

    ctx.check(
        "azimuth bearing uses a vertical continuous axis",
        azimuth_bearing.articulation_type == ArticulationType.CONTINUOUS
        and azimuth_bearing.axis == (0.0, 0.0, 1.0)
        and azimuth_bearing.motion_limits is not None
        and azimuth_bearing.motion_limits.lower is None
        and azimuth_bearing.motion_limits.upper is None,
        details=str(
            {
                "type": azimuth_bearing.articulation_type,
                "axis": azimuth_bearing.axis,
                "limits": None if azimuth_bearing.motion_limits is None else (
                    azimuth_bearing.motion_limits.lower,
                    azimuth_bearing.motion_limits.upper,
                ),
            }
        ),
    )
    ctx.check(
        "sequential leveling joints share the pitch axis",
        shoulder_lift.axis == (0.0, -1.0, 0.0) and bridge_leveling.axis == (0.0, -1.0, 0.0),
        details=f"shoulder={shoulder_lift.axis}, bridge={bridge_leveling.axis}",
    )

    rest_bridge_pos = ctx.part_world_position(bridge_bar)
    with ctx.pose({shoulder_lift: 0.55}):
        lifted_bridge_pos = ctx.part_world_position(bridge_bar)
    ctx.check(
        "positive shoulder lift raises the bino bridge",
        rest_bridge_pos is not None
        and lifted_bridge_pos is not None
        and lifted_bridge_pos[2] > rest_bridge_pos[2] + 0.18,
        details=f"rest={rest_bridge_pos}, lifted={lifted_bridge_pos}",
    )

    with ctx.pose({shoulder_lift: 0.60}):
        tilted_aabb = ctx.part_element_world_aabb(bridge_bar, elem="bridge_crossbar")
    with ctx.pose({shoulder_lift: 0.60, bridge_leveling: -0.60}):
        leveled_aabb = ctx.part_element_world_aabb(bridge_bar, elem="bridge_crossbar")
    tilted_height = None if tilted_aabb is None else tilted_aabb[1][2] - tilted_aabb[0][2]
    leveled_height = None if leveled_aabb is None else leveled_aabb[1][2] - leveled_aabb[0][2]
    ctx.check(
        "bridge leveling joint can restore a nearly horizontal bridge",
        tilted_height is not None
        and leveled_height is not None
        and tilted_height > 0.12
        and leveled_height < 0.05,
        details=f"tilted_height={tilted_height}, leveled_height={leveled_height}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
