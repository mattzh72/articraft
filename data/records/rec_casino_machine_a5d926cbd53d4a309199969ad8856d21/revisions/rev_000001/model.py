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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _build_side_panel_geometry(*, thickness: float) -> object:
    profile = [
        (-0.235, 0.000),
        (0.245, 0.000),
        (0.272, 0.140),
        (0.305, 0.560),
        (0.220, 1.040),
        (0.170, 1.275),
        (0.115, 1.515),
        (0.060, 1.595),
        (-0.080, 1.645),
        (-0.215, 1.520),
        (-0.245, 1.100),
        (-0.245, 0.520),
    ]
    geom = ExtrudeGeometry.from_z0(profile, thickness, cap=True, closed=True)
    geom.rotate_y(math.pi / 2.0)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(-thickness * 0.5, 0.0, 0.0)
    return geom


def _build_guide_ring_mesh(*, outer_radius: float, inner_radius: float, height: float):
    outer_profile = [
        (outer_radius, 0.0),
        (outer_radius, height),
    ]
    inner_profile = [
        (inner_radius, 0.0),
        (inner_radius, height),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        "casino_button_guide_ring",
    )


def _build_console_top_mesh():
    outer_profile = [
        (-0.340, -0.090),
        (0.340, -0.090),
        (0.340, 0.090),
        (-0.340, 0.090),
    ]
    hole_profile = superellipse_profile(0.052, 0.052, exponent=2.0, segments=28)
    left_hole = [(x - 0.100, y + 0.005) for x, y in hole_profile]
    right_hole = [(x + 0.100, y + 0.005) for x, y in hole_profile]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            (left_hole, right_hole),
            0.040,
            center=True,
            cap=True,
            closed=True,
        ),
        "casino_console_top",
    )


def _add_reel(
    model: ArticulatedObject,
    *,
    name: str,
    reel_material,
    hub_material,
):
    reel = model.part(name)
    reel.visual(
        Cylinder(radius=0.085, length=0.112),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=reel_material,
        name="reel_drum",
    )
    reel.visual(
        Cylinder(radius=0.073, length=0.126),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="reel_hub_band",
    )
    reel.visual(
        Cylinder(radius=0.022, length=0.140),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="reel_axle_core",
    )
    reel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.112),
        mass=1.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    return reel


def _add_button(
    model: ArticulatedObject,
    *,
    name: str,
    cap_material,
    trim_material,
):
    button = model.part(name)
    button.visual(
        Cylinder(radius=0.033, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=cap_material,
        name="button_cap",
    )
    button.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=trim_material,
        name="button_skirt",
    )
    button.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=trim_material,
        name="button_stem",
    )
    button.inertial = Inertial.from_geometry(
        Box((0.066, 0.066, 0.056)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    return button


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="casino_slot_machine")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.17, 0.06, 0.20, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.08, 0.08, 0.10, 1.0))
    chrome = model.material("chrome", rgba=(0.76, 0.78, 0.82, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.25, 0.33, 0.38, 0.30))
    marquee_gold = model.material("marquee_gold", rgba=(0.86, 0.65, 0.20, 1.0))
    reel_cream = model.material("reel_cream", rgba=(0.92, 0.91, 0.83, 1.0))
    reel_gold = model.material("reel_gold", rgba=(0.78, 0.68, 0.34, 1.0))
    lamp_red = model.material("lamp_red", rgba=(0.85, 0.16, 0.16, 1.0))
    lamp_green = model.material("lamp_green", rgba=(0.20, 0.80, 0.34, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    cabinet = model.part("cabinet")

    side_mesh = mesh_from_geometry(
        _build_side_panel_geometry(thickness=0.030),
        "casino_side_panel",
    )
    console_top_mesh = _build_console_top_mesh()
    guide_ring_mesh = _build_guide_ring_mesh(
        outer_radius=0.039,
        inner_radius=0.0265,
        height=0.020,
    )

    cabinet.visual(
        side_mesh,
        origin=Origin(xyz=(-0.345, 0.0, 0.0)),
        material=cabinet_paint,
        name="left_side_shell",
    )
    cabinet.visual(
        side_mesh,
        origin=Origin(xyz=(0.345, 0.0, 0.0)),
        material=cabinet_paint,
        name="right_side_shell",
    )
    cabinet.visual(
        Box((0.700, 0.520, 0.040)),
        origin=Origin(xyz=(0.0, 0.000, 0.020)),
        material=cabinet_shadow,
        name="base_floor",
    )
    cabinet.visual(
        Box((0.700, 0.030, 1.180)),
        origin=Origin(xyz=(0.0, -0.235, 0.590)),
        material=cabinet_paint,
        name="rear_panel_lower",
    )
    cabinet.visual(
        Box((0.660, 0.090, 0.500)),
        origin=Origin(xyz=(0.0, 0.010, 0.870)),
        material=cabinet_shadow,
        name="reel_bay_backer",
    )
    cabinet.visual(
        Box((0.640, 0.100, 0.150)),
        origin=Origin(xyz=(0.0, 0.180, 0.510)),
        material=cabinet_paint,
        name="console_nose",
    )
    cabinet.visual(
        console_top_mesh,
        origin=Origin(xyz=(0.0, 0.160, 0.600)),
        material=cabinet_paint,
        name="console_top",
    )
    cabinet.visual(
        Box((0.680, 0.028, 0.055)),
        origin=Origin(xyz=(0.0, 0.262, 0.665)),
        material=chrome,
        name="window_bezel_bottom",
    )
    cabinet.visual(
        Box((0.680, 0.028, 0.055)),
        origin=Origin(xyz=(0.0, 0.262, 1.095)),
        material=chrome,
        name="window_bezel_top",
    )
    cabinet.visual(
        Box((0.072, 0.028, 0.430)),
        origin=Origin(xyz=(-0.304, 0.262, 0.880)),
        material=chrome,
        name="window_bezel_left",
    )
    cabinet.visual(
        Box((0.072, 0.028, 0.430)),
        origin=Origin(xyz=(0.304, 0.262, 0.880)),
        material=chrome,
        name="window_bezel_right",
    )
    cabinet.visual(
        Box((0.560, 0.012, 0.380)),
        origin=Origin(xyz=(0.0, 0.247, 0.880)),
        material=smoked_glass,
        name="window_lens",
    )
    cabinet.visual(
        Box((0.600, 0.080, 0.140)),
        origin=Origin(xyz=(0.0, 0.160, 1.165)),
        material=cabinet_shadow,
        name="header_panel",
    )
    cabinet.visual(
        Box((0.660, 0.220, 0.250)),
        origin=Origin(xyz=(0.0, 0.030, 1.310)),
        material=cabinet_paint,
        name="marquee_box",
    )
    cabinet.visual(
        Cylinder(radius=0.140, length=0.660),
        origin=Origin(xyz=(0.0, 0.030, 1.470), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cabinet_paint,
        name="marquee_roof",
    )
    cabinet.visual(
        Box((0.560, 0.020, 0.170)),
        origin=Origin(xyz=(0.0, 0.160, 1.315)),
        material=marquee_gold,
        name="marquee_face",
    )
    cabinet.visual(
        Box((0.680, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, 0.262, 0.100)),
        material=chrome,
        name="door_frame_bottom",
    )
    cabinet.visual(
        Box((0.680, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, 0.262, 0.500)),
        material=chrome,
        name="door_frame_top",
    )
    cabinet.visual(
        Box((0.110, 0.028, 0.400)),
        origin=Origin(xyz=(-0.285, 0.262, 0.300)),
        material=chrome,
        name="door_frame_left",
    )
    cabinet.visual(
        Box((0.110, 0.028, 0.400)),
        origin=Origin(xyz=(0.285, 0.262, 0.300)),
        material=chrome,
        name="door_frame_right",
    )
    cabinet.visual(
        guide_ring_mesh,
        origin=Origin(xyz=(-0.100, 0.165, 0.620)),
        material=chrome,
        name="left_button_guide",
    )
    cabinet.visual(
        guide_ring_mesh,
        origin=Origin(xyz=(0.100, 0.165, 0.620)),
        material=chrome,
        name="right_button_guide",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.720, 0.560, 1.660)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.015, 0.830)),
    )

    left_reel = _add_reel(
        model,
        name="left_reel",
        reel_material=reel_cream,
        hub_material=reel_gold,
    )
    center_reel = _add_reel(
        model,
        name="center_reel",
        reel_material=reel_cream,
        hub_material=reel_gold,
    )
    right_reel = _add_reel(
        model,
        name="right_reel",
        reel_material=reel_cream,
        hub_material=reel_gold,
    )

    left_button = _add_button(
        model,
        name="left_button",
        cap_material=lamp_red,
        trim_material=chrome,
    )
    right_button = _add_button(
        model,
        name="right_button",
        cap_material=lamp_green,
        trim_material=chrome,
    )

    cashbox_door = model.part("cashbox_door")
    cashbox_door.visual(
        Box((0.460, 0.018, 0.360)),
        origin=Origin(xyz=(0.230, -0.009, 0.180)),
        material=cabinet_paint,
        name="door_panel",
    )
    cashbox_door.visual(
        Box((0.380, 0.008, 0.280)),
        origin=Origin(xyz=(0.230, -0.001, 0.180)),
        material=cabinet_shadow,
        name="door_inset",
    )
    cashbox_door.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.400, 0.010, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="door_pull",
    )
    cashbox_door.inertial = Inertial.from_geometry(
        Box((0.460, 0.020, 0.360)),
        mass=4.5,
        origin=Origin(xyz=(0.230, -0.010, 0.180)),
    )

    model.articulation(
        "cabinet_to_left_reel",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=left_reel,
        origin=Origin(xyz=(-0.180, 0.135, 0.880)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "cabinet_to_center_reel",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=center_reel,
        origin=Origin(xyz=(0.000, 0.135, 0.880)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "cabinet_to_right_reel",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=right_reel,
        origin=Origin(xyz=(0.180, 0.135, 0.880)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "cabinet_to_left_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=left_button,
        origin=Origin(xyz=(-0.100, 0.165, 0.620)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.15,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "cabinet_to_right_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=right_button,
        origin=Origin(xyz=(0.100, 0.165, 0.620)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.15,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "cabinet_to_cashbox_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cashbox_door,
        origin=Origin(xyz=(-0.230, 0.277, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=1.35,
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

    cabinet = object_model.get_part("cabinet")
    left_reel = object_model.get_part("left_reel")
    center_reel = object_model.get_part("center_reel")
    right_reel = object_model.get_part("right_reel")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")
    cashbox_door = object_model.get_part("cashbox_door")

    left_reel_joint = object_model.get_articulation("cabinet_to_left_reel")
    center_reel_joint = object_model.get_articulation("cabinet_to_center_reel")
    right_reel_joint = object_model.get_articulation("cabinet_to_right_reel")
    left_button_joint = object_model.get_articulation("cabinet_to_left_button")
    right_button_joint = object_model.get_articulation("cabinet_to_right_button")
    door_joint = object_model.get_articulation("cabinet_to_cashbox_door")

    ctx.check(
        "reels use continuous horizontal spin joints",
        all(
            joint.joint_type == ArticulationType.CONTINUOUS and joint.axis == (1.0, 0.0, 0.0)
            for joint in (left_reel_joint, center_reel_joint, right_reel_joint)
        ),
        details=(
            f"axes={[left_reel_joint.axis, center_reel_joint.axis, right_reel_joint.axis]}, "
            f"types={[left_reel_joint.joint_type, center_reel_joint.joint_type, right_reel_joint.joint_type]}"
        ),
    )
    ctx.check(
        "front buttons use downward prismatic travel",
        all(
            joint.joint_type == ArticulationType.PRISMATIC and joint.axis == (0.0, 0.0, -1.0)
            for joint in (left_button_joint, right_button_joint)
        ),
        details=(
            f"axes={[left_button_joint.axis, right_button_joint.axis]}, "
            f"types={[left_button_joint.joint_type, right_button_joint.joint_type]}"
        ),
    )
    ctx.check(
        "cashbox door uses a vertical side hinge",
        door_joint.joint_type == ArticulationType.REVOLUTE and door_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={door_joint.joint_type}, axis={door_joint.axis}",
    )

    ctx.expect_within(
        left_reel,
        cabinet,
        axes="xz",
        inner_elem="reel_drum",
        outer_elem="window_lens",
        margin=0.015,
        name="left reel sits within reel window",
    )
    ctx.expect_within(
        center_reel,
        cabinet,
        axes="xz",
        inner_elem="reel_drum",
        outer_elem="window_lens",
        margin=0.015,
        name="center reel sits within reel window",
    )
    ctx.expect_within(
        right_reel,
        cabinet,
        axes="xz",
        inner_elem="reel_drum",
        outer_elem="window_lens",
        margin=0.015,
        name="right reel sits within reel window",
    )
    ctx.expect_within(
        left_button,
        cabinet,
        axes="xy",
        inner_elem="button_skirt",
        outer_elem="left_button_guide",
        margin=0.002,
        name="left button stays centered in its guide",
    )
    ctx.expect_within(
        right_button,
        cabinet,
        axes="xy",
        inner_elem="button_skirt",
        outer_elem="right_button_guide",
        margin=0.002,
        name="right button stays centered in its guide",
    )

    left_rest = ctx.part_world_position(left_button)
    right_rest = ctx.part_world_position(right_button)
    with ctx.pose({left_button_joint: 0.006, right_button_joint: 0.006}):
        left_pressed = ctx.part_world_position(left_button)
        right_pressed = ctx.part_world_position(right_button)

    ctx.check(
        "buttons depress into the console",
        left_rest is not None
        and right_rest is not None
        and left_pressed is not None
        and right_pressed is not None
        and left_pressed[2] < left_rest[2] - 0.004
        and right_pressed[2] < right_rest[2] - 0.004,
        details=(
            f"left_rest={left_rest}, left_pressed={left_pressed}, "
            f"right_rest={right_rest}, right_pressed={right_pressed}"
        ),
    )

    closed_aabb = ctx.part_world_aabb(cashbox_door)
    with ctx.pose({door_joint: 1.15}):
        open_aabb = ctx.part_world_aabb(cashbox_door)

    ctx.check(
        "cashbox door swings outward from the cabinet",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.12,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
