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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    half_height = height * 0.5
    return [
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height),
    ]


def _rect_frame_mesh(
    *,
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    depth: float,
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        _rect_profile(outer_width, outer_height),
        [_rect_profile(inner_width, inner_height)],
        depth,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _joint_kind_name(joint) -> str:
    joint_type = getattr(joint, "articulation_type", None)
    if joint_type is None:
        joint_type = getattr(joint, "joint_type", None)
    return getattr(joint_type, "name", str(joint_type))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_microwave")

    housing_white = model.material("housing_white", rgba=(0.94, 0.95, 0.93, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))
    control_charcoal = model.material("control_charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    cavity_steel = model.material("cavity_steel", rgba=(0.78, 0.79, 0.80, 1.0))
    glass = model.material("glass", rgba=(0.68, 0.81, 0.88, 0.30))
    dark_glass = model.material("dark_glass", rgba=(0.20, 0.28, 0.34, 0.34))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.33, 0.34, 0.35, 1.0))

    outer_width = 0.490
    outer_depth = 0.390
    outer_height = 0.290
    shell_thickness = 0.020

    door_width = 0.332
    door_height = 0.224
    door_thickness = 0.026
    door_left_x = -0.227
    door_center_z = 0.148

    control_strip_width = 0.140
    control_strip_center_x = outer_width * 0.5 - control_strip_width * 0.5

    housing = model.part("housing")
    housing.visual(
        Box((shell_thickness, outer_depth, outer_height)),
        origin=Origin(xyz=(-outer_width * 0.5 + shell_thickness * 0.5, 0.0, outer_height * 0.5)),
        material=housing_white,
        name="left_wall",
    )
    housing.visual(
        Box((outer_width, outer_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, shell_thickness * 0.5)),
        material=housing_white,
        name="bottom_shell",
    )
    housing.visual(
        Box((outer_width, outer_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, outer_height - shell_thickness * 0.5)),
        material=housing_white,
        name="top_shell",
    )
    housing.visual(
        Box((outer_width, shell_thickness, outer_height)),
        origin=Origin(xyz=(0.0, outer_depth * 0.5 - shell_thickness * 0.5, outer_height * 0.5)),
        material=housing_white,
        name="back_panel",
    )
    housing.visual(
        Box((control_strip_width, outer_depth, outer_height)),
        origin=Origin(xyz=(control_strip_center_x, 0.0, outer_height * 0.5)),
        material=housing_white,
        name="right_bay",
    )
    housing.visual(
        Box((0.022, 0.028, 0.244)),
        origin=Origin(xyz=(door_left_x - 0.007, -outer_depth * 0.5 + 0.014, door_center_z)),
        material=trim_black,
        name="door_side_bezel",
    )
    housing.visual(
        Box((0.352, 0.028, 0.018)),
        origin=Origin(xyz=(-0.055, -outer_depth * 0.5 + 0.014, 0.269)),
        material=trim_black,
        name="front_top_bezel",
    )
    housing.visual(
        Box((0.352, 0.028, 0.016)),
        origin=Origin(xyz=(-0.055, -outer_depth * 0.5 + 0.014, 0.028)),
        material=trim_black,
        name="front_bottom_bezel",
    )
    housing.visual(
        Box((0.012, 0.028, 0.240)),
        origin=Origin(xyz=(0.099, -outer_depth * 0.5 + 0.014, 0.148)),
        material=trim_black,
        name="door_control_divider",
    )
    housing.visual(
        Box((0.132, 0.014, 0.238)),
        origin=Origin(xyz=(0.175, -outer_depth * 0.5 + 0.007, 0.145)),
        material=control_charcoal,
        name="control_panel_face",
    )
    housing.inertial = Inertial.from_geometry(
        Box((outer_width, outer_depth, outer_height)),
        mass=10.8,
        origin=Origin(xyz=(0.0, 0.0, outer_height * 0.5)),
    )

    cavity_outer_width = 0.320
    cavity_outer_height = 0.230
    cavity_inner_width = 0.300
    cavity_inner_height = 0.210
    cavity_depth = 0.310
    cavity_center = (-0.061, -0.011, 0.135)

    cavity = model.part("cavity")
    cavity.visual(
        Box((0.010, cavity_depth, cavity_outer_height)),
        origin=Origin(xyz=(-cavity_outer_width * 0.5 + 0.005, 0.0, 0.0)),
        material=cavity_steel,
        name="cavity_left_wall",
    )
    cavity.visual(
        Box((0.010, cavity_depth, cavity_outer_height)),
        origin=Origin(xyz=(cavity_outer_width * 0.5 - 0.005, 0.0, 0.0)),
        material=cavity_steel,
        name="cavity_right_wall",
    )
    cavity.visual(
        Box((cavity_inner_width, cavity_depth, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -cavity_outer_height * 0.5 + 0.005)),
        material=cavity_steel,
        name="cavity_floor",
    )
    cavity.visual(
        Box((cavity_inner_width, cavity_depth, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, cavity_outer_height * 0.5 - 0.005)),
        material=cavity_steel,
        name="cavity_ceiling",
    )
    cavity.visual(
        Box((cavity_outer_width, 0.010, cavity_outer_height)),
        origin=Origin(xyz=(0.0, cavity_depth * 0.5 - 0.005, 0.0)),
        material=cavity_steel,
        name="cavity_back",
    )
    cavity.inertial = Inertial.from_geometry(
        Box((cavity_outer_width, cavity_depth + 0.010, cavity_outer_height)),
        mass=3.9,
    )

    model.articulation(
        "housing_to_cavity",
        ArticulationType.FIXED,
        parent=housing,
        child=cavity,
        origin=Origin(xyz=cavity_center),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=hub_gray,
        name="hub_stem",
    )
    hub.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=hub_gray,
        name="hub_cap",
    )
    hub.inertial = Inertial.from_geometry(
        Box((0.026, 0.026, 0.016)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "cavity_to_hub",
        ArticulationType.FIXED,
        parent=cavity,
        child=hub,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.124, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=glass,
        name="tray_disc",
    )
    turntable.visual(
        Cylinder(radius=0.023, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=glass,
        name="tray_hub_puck",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.124, length=0.006),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )
    model.articulation(
        "hub_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=hub,
        child=turntable,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    door = model.part("door")
    door_frame_mesh = _rect_frame_mesh(
        outer_width=door_width,
        outer_height=door_height,
        inner_width=0.248,
        inner_height=0.156,
        depth=door_thickness,
        name="microwave_door_frame",
    )
    door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(door_width * 0.5, -door_thickness * 0.5, 0.0)),
        material=trim_black,
        name="door_frame",
    )
    door.visual(
        Box((0.256, 0.006, 0.164)),
        origin=Origin(xyz=(door_width * 0.5, -0.010, 0.0)),
        material=dark_glass,
        name="door_window",
    )
    door.visual(
        Box((0.020, 0.016, 0.142)),
        origin=Origin(xyz=(door_width - 0.024, -door_thickness - 0.008, 0.0)),
        material=trim_black,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.042, door_height)),
        mass=2.6,
        origin=Origin(xyz=(door_width * 0.5, -0.021, 0.0)),
    )
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(door_left_x, -outer_depth * 0.5, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.95,
        ),
    )

    def add_dial(
        *,
        part_name: str,
        joint_name: str,
        center_z: float,
        mesh_tag: str,
    ) -> None:
        dial = model.part(part_name)
        dial.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name=f"{mesh_tag}_skirt",
        )
        dial.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name=f"{mesh_tag}_body",
        )
        dial.visual(
            Box((0.003, 0.004, 0.008)),
            origin=Origin(xyz=(0.0, -0.032, 0.014)),
            material=housing_white,
            name=f"{mesh_tag}_pointer",
        )
        dial.inertial = Inertial.from_geometry(
            Box((0.050, 0.036, 0.050)),
            mass=0.06,
            origin=Origin(xyz=(0.0, -0.018, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=dial,
            origin=Origin(xyz=(0.175, -outer_depth * 0.5, center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=6.0),
        )

    add_dial(
        part_name="upper_dial",
        joint_name="housing_to_upper_dial",
        center_z=0.188,
        mesh_tag="upper_dial",
    )
    add_dial(
        part_name="lower_dial",
        joint_name="housing_to_lower_dial",
        center_z=0.102,
        mesh_tag="lower_dial",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cavity = object_model.get_part("cavity")
    hub = object_model.get_part("hub")
    turntable = object_model.get_part("turntable")
    door = object_model.get_part("door")
    upper_dial = object_model.get_part("upper_dial")
    lower_dial = object_model.get_part("lower_dial")

    door_hinge = object_model.get_articulation("housing_to_door")
    tray_spin = object_model.get_articulation("hub_to_turntable")
    upper_dial_spin = object_model.get_articulation("housing_to_upper_dial")
    lower_dial_spin = object_model.get_articulation("housing_to_lower_dial")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "door_joint_is_vertical_revolute",
        _joint_kind_name(door_hinge) == "REVOLUTE" and door_hinge.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical revolute hinge, got type={_joint_kind_name(door_hinge)} axis={door_hinge.axis}",
    )
    ctx.check(
        "door_opening_range_is_realistic",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper >= 1.8,
        details=f"door limits={getattr(door_hinge, 'motion_limits', None)}",
    )
    ctx.check(
        "turntable_joint_is_vertical_continuous",
        _joint_kind_name(tray_spin) == "CONTINUOUS" and tray_spin.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical continuous turntable axis, got type={_joint_kind_name(tray_spin)} axis={tray_spin.axis}",
    )
    ctx.check(
        "dial_joints_are_continuous_fore_aft",
        _joint_kind_name(upper_dial_spin) == "CONTINUOUS"
        and _joint_kind_name(lower_dial_spin) == "CONTINUOUS"
        and upper_dial_spin.axis == (0.0, -1.0, 0.0)
        and lower_dial_spin.axis == (0.0, -1.0, 0.0),
        details=(
            f"upper={_joint_kind_name(upper_dial_spin)} {upper_dial_spin.axis}, "
            f"lower={_joint_kind_name(lower_dial_spin)} {lower_dial_spin.axis}"
        ),
    )

    ctx.expect_contact(cavity, housing, name="cavity_is_supported_inside_housing", contact_tol=1e-6)
    ctx.expect_contact(upper_dial, housing, name="upper_dial_is_mounted_to_control_strip", contact_tol=1e-6)
    ctx.expect_contact(lower_dial, housing, name="lower_dial_is_mounted_to_control_strip", contact_tol=1e-6)

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            housing,
            door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed_door_sits_flush_to_front_face",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="xz",
            min_overlap=0.20,
            name="door_covers_the_front_opening",
        )

    with ctx.pose({door_hinge: 1.35}):
        ctx.expect_gap(
            housing,
            door,
            axis="x",
            positive_elem="control_panel_face",
            negative_elem="door_frame",
            min_gap=0.03,
            name="open_door_clears_control_strip",
        )

    with ctx.pose({tray_spin: 2.10}):
        ctx.expect_contact(turntable, hub, name="glass_turntable_stays_seated_on_hub", contact_tol=1e-6)
        ctx.expect_within(
            turntable,
            cavity,
            axes="xy",
            margin=0.0,
            name="turntable_stays_inside_cavity_footprint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
