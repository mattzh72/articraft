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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_column_candy_vending_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.76, 0.14, 0.16, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.78, 0.80, 0.84, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.72, 0.82, 0.88, 0.34))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    flap_clear = model.material("flap_clear", rgba=(0.76, 0.84, 0.90, 0.48))
    dark_shadow = model.material("dark_shadow", rgba=(0.20, 0.20, 0.22, 1.0))

    def _canister_shell_mesh(name: str, *, radius: float, wall: float, height: float):
        shell = LatheGeometry.from_shell_profiles(
            [
                (radius * 0.76, 0.016),
                (radius * 0.96, 0.022),
                (radius, 0.036),
                (radius, height - 0.022),
                (radius * 0.92, height - 0.008),
                (radius * 0.78, height),
            ],
            [
                (radius * 0.76 - wall, 0.020),
                (radius * 0.96 - wall, 0.026),
                (radius - wall, 0.040),
                (radius - wall, height - 0.028),
                (radius * 0.92 - wall, height - 0.014),
                (radius * 0.78 - wall, height - 0.004),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        )
        return mesh_from_geometry(shell, name)

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.360, 0.220, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=cabinet_red,
        name="cabinet_body",
    )
    cabinet.visual(
        Box((0.340, 0.200, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=trim_silver,
        name="top_deck_trim",
    )
    cabinet.visual(
        Box((0.346, 0.210, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_shadow,
        name="base_shadow_band",
    )
    cabinet.visual(
        Cylinder(radius=0.066, length=0.010),
        origin=Origin(xyz=(-0.095, 0.000, 0.185)),
        material=trim_silver,
        name="left_canister_pedestal",
    )
    cabinet.visual(
        Cylinder(radius=0.066, length=0.010),
        origin=Origin(xyz=(0.095, 0.000, 0.185)),
        material=trim_silver,
        name="right_canister_pedestal",
    )
    cabinet.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(-0.094, -0.114, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="left_knob_boss",
    )
    cabinet.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.094, -0.114, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="right_knob_boss",
    )
    cabinet.visual(
        Box((0.124, 0.008, 0.070)),
        origin=Origin(xyz=(0.0, -0.106, 0.083)),
        material=trim_silver,
        name="chute_back_wall",
    )
    cabinet.visual(
        Box((0.124, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, -0.135, 0.113)),
        material=trim_silver,
        name="chute_roof",
    )
    cabinet.visual(
        Box((0.008, 0.050, 0.068)),
        origin=Origin(xyz=(-0.058, -0.135, 0.084)),
        material=trim_silver,
        name="chute_left_wall",
    )
    cabinet.visual(
        Box((0.008, 0.050, 0.068)),
        origin=Origin(xyz=(0.058, -0.135, 0.084)),
        material=trim_silver,
        name="chute_right_wall",
    )
    cabinet.visual(
        Box((0.124, 0.038, 0.010)),
        origin=Origin(xyz=(0.0, -0.129, 0.045)),
        material=trim_silver,
        name="chute_floor",
    )
    cabinet.visual(
        Box((0.124, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.154, 0.046)),
        material=trim_silver,
        name="chute_lip",
    )
    cabinet.visual(
        Box((0.180, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, -0.103, 0.138)),
        material=trim_silver,
        name="selector_panel_band",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.360, 0.220, 0.190)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    left_canister = model.part("left_canister")
    left_canister.visual(
        Cylinder(radius=0.060, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=trim_silver,
        name="base_collar",
    )
    left_canister.visual(
        _canister_shell_mesh(
            "left_canister_shell",
            radius=0.056,
            wall=0.004,
            height=0.272,
        ),
        material=smoked_clear,
        name="canister_shell",
    )
    left_canister.visual(
        Cylinder(radius=0.059, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.279)),
        material=trim_silver,
        name="top_collar",
    )
    left_canister.visual(
        Cylinder(radius=0.040, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.294)),
        material=trim_silver,
        name="lid_cap",
    )
    left_canister.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        material=trim_silver,
        name="lid_handle",
    )
    left_canister.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.320),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    right_canister = model.part("right_canister")
    right_canister.visual(
        Cylinder(radius=0.060, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=trim_silver,
        name="base_collar",
    )
    right_canister.visual(
        _canister_shell_mesh(
            "right_canister_shell",
            radius=0.056,
            wall=0.004,
            height=0.272,
        ),
        material=smoked_clear,
        name="canister_shell",
    )
    right_canister.visual(
        Cylinder(radius=0.059, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.279)),
        material=trim_silver,
        name="top_collar",
    )
    right_canister.visual(
        Cylinder(radius=0.040, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.294)),
        material=trim_silver,
        name="lid_cap",
    )
    right_canister.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        material=trim_silver,
        name="lid_handle",
    )
    right_canister.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.320),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    left_selector = model.part("left_selector")
    left_selector.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_shadow,
        name="selector_shaft",
    )
    left_selector.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_shadow,
        name="selector_hub",
    )
    left_selector.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, -0.037, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="selector_disc",
    )
    left_selector.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.047, 0.014)),
        material=trim_silver,
        name="selector_pointer",
    )
    left_selector.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.050)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.030, 0.005)),
    )

    right_selector = model.part("right_selector")
    right_selector.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_shadow,
        name="selector_shaft",
    )
    right_selector.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_shadow,
        name="selector_hub",
    )
    right_selector.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, -0.037, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="selector_disc",
    )
    right_selector.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.047, 0.014)),
        material=trim_silver,
        name="selector_pointer",
    )
    right_selector.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.050)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.030, 0.005)),
    )

    chute_flap = model.part("chute_flap")
    chute_flap.visual(
        Box((0.102, 0.003, 0.052)),
        origin=Origin(xyz=(0.0, -0.0015, 0.026)),
        material=flap_clear,
        name="flap_panel",
    )
    chute_flap.visual(
        Cylinder(radius=0.004, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_silver,
        name="flap_hinge_barrel",
    )
    chute_flap.inertial = Inertial.from_geometry(
        Box((0.110, 0.010, 0.060)),
        mass=0.06,
        origin=Origin(xyz=(0.0, -0.002, 0.026)),
    )

    model.articulation(
        "cabinet_to_left_canister",
        ArticulationType.FIXED,
        parent=cabinet,
        child=left_canister,
        origin=Origin(xyz=(-0.095, 0.000, 0.190)),
    )
    model.articulation(
        "cabinet_to_right_canister",
        ArticulationType.FIXED,
        parent=cabinet,
        child=right_canister,
        origin=Origin(xyz=(0.095, 0.000, 0.190)),
    )
    model.articulation(
        "cabinet_to_left_selector",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=left_selector,
        origin=Origin(xyz=(-0.094, -0.118, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    model.articulation(
        "cabinet_to_right_selector",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=right_selector,
        origin=Origin(xyz=(0.094, -0.118, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    model.articulation(
        "cabinet_to_chute_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=chute_flap,
        origin=Origin(xyz=(0.0, -0.158, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=3.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_canister = object_model.get_part("left_canister")
    right_canister = object_model.get_part("right_canister")
    left_selector = object_model.get_part("left_selector")
    right_selector = object_model.get_part("right_selector")
    chute_flap = object_model.get_part("chute_flap")

    left_selector_joint = object_model.get_articulation("cabinet_to_left_selector")
    right_selector_joint = object_model.get_articulation("cabinet_to_right_selector")
    flap_joint = object_model.get_articulation("cabinet_to_chute_flap")

    ctx.expect_contact(
        left_canister,
        cabinet,
        elem_a="base_collar",
        elem_b="left_canister_pedestal",
        name="left canister is seated on its pedestal",
    )
    ctx.expect_contact(
        right_canister,
        cabinet,
        elem_a="base_collar",
        elem_b="right_canister_pedestal",
        name="right canister is seated on its pedestal",
    )
    ctx.expect_contact(
        left_selector,
        cabinet,
        elem_a="selector_shaft",
        elem_b="left_knob_boss",
        name="left selector shaft mounts to the cabinet",
    )
    ctx.expect_contact(
        right_selector,
        cabinet,
        elem_a="selector_shaft",
        elem_b="right_knob_boss",
        name="right selector shaft mounts to the cabinet",
    )

    ctx.check(
        "left selector articulation is continuous on a horizontal shaft",
        left_selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in left_selector_joint.axis) == (0.0, 1.0, 0.0)
        and left_selector_joint.motion_limits is not None
        and left_selector_joint.motion_limits.lower is None
        and left_selector_joint.motion_limits.upper is None,
        details=f"type={left_selector_joint.articulation_type}, axis={left_selector_joint.axis}, limits={left_selector_joint.motion_limits}",
    )
    ctx.check(
        "right selector articulation is continuous on a horizontal shaft",
        right_selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in right_selector_joint.axis) == (0.0, 1.0, 0.0)
        and right_selector_joint.motion_limits is not None
        and right_selector_joint.motion_limits.lower is None
        and right_selector_joint.motion_limits.upper is None,
        details=f"type={right_selector_joint.articulation_type}, axis={right_selector_joint.axis}, limits={right_selector_joint.motion_limits}",
    )

    left_pointer_closed = ctx.part_element_world_aabb(left_selector, elem="selector_pointer")
    with ctx.pose({left_selector_joint: math.pi / 2.0}):
        left_pointer_turned = ctx.part_element_world_aabb(left_selector, elem="selector_pointer")
    if left_pointer_closed is not None and left_pointer_turned is not None:
        closed_center = (
            (left_pointer_closed[0][0] + left_pointer_closed[1][0]) * 0.5,
            (left_pointer_closed[0][1] + left_pointer_closed[1][1]) * 0.5,
            (left_pointer_closed[0][2] + left_pointer_closed[1][2]) * 0.5,
        )
        turned_center = (
            (left_pointer_turned[0][0] + left_pointer_turned[1][0]) * 0.5,
            (left_pointer_turned[0][1] + left_pointer_turned[1][1]) * 0.5,
            (left_pointer_turned[0][2] + left_pointer_turned[1][2]) * 0.5,
        )
        ctx.check(
            "left selector pointer visibly rotates around the shaft",
            turned_center[0] > closed_center[0] + 0.010 and turned_center[2] < closed_center[2] - 0.010,
            details=f"closed_center={closed_center}, turned_center={turned_center}",
        )
    else:
        ctx.fail("left selector pointer visibly rotates around the shaft", "selector pointer AABB unavailable")

    ctx.check(
        "collection flap hinge is a lower horizontal revolute joint",
        flap_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in flap_joint.axis) == (1.0, 0.0, 0.0)
        and flap_joint.motion_limits is not None
        and flap_joint.motion_limits.lower == 0.0
        and flap_joint.motion_limits.upper is not None
        and flap_joint.motion_limits.upper >= 1.2,
        details=f"type={flap_joint.articulation_type}, axis={flap_joint.axis}, limits={flap_joint.motion_limits}",
    )

    flap_closed = ctx.part_element_world_aabb(chute_flap, elem="flap_panel")
    with ctx.pose({flap_joint: 1.20}):
        flap_open = ctx.part_element_world_aabb(chute_flap, elem="flap_panel")
    if flap_closed is not None and flap_open is not None:
        closed_tip = flap_closed[1]
        open_front = flap_open[0]
        ctx.check(
            "collection flap swings outward and downward",
            open_front[1] < flap_closed[0][1] - 0.015 and flap_open[0][2] < closed_tip[2] - 0.015,
            details=f"closed={flap_closed}, open={flap_open}",
        )
    else:
        ctx.fail("collection flap swings outward and downward", "flap panel AABB unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
