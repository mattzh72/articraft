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
    model = ArticulatedObject(name="roof_remote_weapon_station")

    roof_gray = model.material("roof_gray", rgba=(0.30, 0.32, 0.34, 1.0))
    armor_green = model.material("armor_green", rgba=(0.43, 0.47, 0.35, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    barrel_black = model.material("barrel_black", rgba=(0.07, 0.07, 0.08, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.25, 0.26, 0.27, 1.0))

    roof_base = model.part("roof_base")
    roof_base.visual(
        Box((2.20, 1.80, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=roof_gray,
        name="roof_panel",
    )
    roof_base.visual(
        Box((0.72, 0.54, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=dark_metal,
        name="station_plinth",
    )
    roof_base.visual(
        Cylinder(radius=0.34, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=dark_metal,
        name="mount_collar",
    )
    roof_base.visual(
        Cylinder(radius=0.40, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=hinge_gray,
        name="bearing_race",
    )
    roof_base.inertial = Inertial.from_geometry(
        Box((2.20, 1.80, 0.18)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    turret_shell = model.part("turret_shell")
    turret_shell.visual(
        Cylinder(radius=0.38, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_metal,
        name="yaw_ring",
    )
    turret_shell.visual(
        Cylinder(radius=0.28, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=dark_metal,
        name="ring_skirt",
    )
    turret_shell.visual(
        Box((0.50, 0.40, 0.26)),
        origin=Origin(xyz=(-0.10, 0.0, 0.28)),
        material=armor_green,
        name="core_body",
    )
    turret_shell.visual(
        Box((0.24, 0.34, 0.18)),
        origin=Origin(xyz=(-0.38, 0.0, 0.24)),
        material=armor_green,
        name="rear_housing",
    )
    turret_shell.visual(
        Box((0.54, 0.44, 0.03)),
        origin=Origin(xyz=(-0.07, 0.0, 0.420)),
        material=armor_green,
        name="turret_roof",
    )
    turret_shell.visual(
        Box((0.44, 0.03, 0.28)),
        origin=Origin(xyz=(-0.04, 0.225, 0.305), rpy=(0.20, 0.0, 0.0)),
        material=armor_green,
        name="left_armor",
    )
    turret_shell.visual(
        Box((0.44, 0.03, 0.28)),
        origin=Origin(xyz=(-0.04, -0.225, 0.305), rpy=(-0.20, 0.0, 0.0)),
        material=armor_green,
        name="right_armor",
    )
    turret_shell.visual(
        Box((0.20, 0.30, 0.16)),
        origin=Origin(xyz=(0.14, 0.0, 0.31)),
        material=armor_green,
        name="forward_hood",
    )
    turret_shell.visual(
        Box((0.14, 0.09, 0.18)),
        origin=Origin(xyz=(0.29, 0.215, 0.29)),
        material=armor_green,
        name="left_trunnion_cheek",
    )
    turret_shell.visual(
        Box((0.14, 0.09, 0.18)),
        origin=Origin(xyz=(0.29, -0.215, 0.29)),
        material=armor_green,
        name="right_trunnion_cheek",
    )
    turret_shell.inertial = Inertial.from_geometry(
        Box((1.00, 0.90, 0.60)),
        mass=185.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.28)),
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        Cylinder(radius=0.045, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_shaft",
    )
    weapon_cradle.visual(
        Box((0.26, 0.18, 0.14)),
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        material=dark_metal,
        name="receiver_block",
    )
    weapon_cradle.visual(
        Box((0.62, 0.30, 0.024)),
        origin=Origin(xyz=(0.26, 0.0, -0.118)),
        material=armor_green,
        name="bottom_pan",
    )
    weapon_cradle.visual(
        Box((0.60, 0.024, 0.24)),
        origin=Origin(xyz=(0.26, 0.148, 0.0)),
        material=armor_green,
        name="left_wall",
    )
    weapon_cradle.visual(
        Box((0.60, 0.024, 0.24)),
        origin=Origin(xyz=(0.26, -0.148, 0.0)),
        material=armor_green,
        name="right_wall",
    )
    weapon_cradle.visual(
        Box((0.036, 0.30, 0.20)),
        origin=Origin(xyz=(-0.038, 0.0, -0.010)),
        material=armor_green,
        name="rear_wall",
    )
    weapon_cradle.visual(
        Box((0.040, 0.30, 0.09)),
        origin=Origin(xyz=(0.56, 0.0, -0.075)),
        material=armor_green,
        name="front_lower_bridge",
    )
    weapon_cradle.visual(
        Box((0.040, 0.070, 0.11)),
        origin=Origin(xyz=(0.56, 0.115, 0.045)),
        material=armor_green,
        name="front_left_cheek",
    )
    weapon_cradle.visual(
        Box((0.040, 0.070, 0.11)),
        origin=Origin(xyz=(0.56, -0.115, 0.045)),
        material=armor_green,
        name="front_right_cheek",
    )
    weapon_cradle.visual(
        Box((0.052, 0.28, 0.026)),
        origin=Origin(xyz=(-0.022, 0.0, 0.108)),
        material=armor_green,
        name="rear_rim",
    )
    weapon_cradle.visual(
        Box((0.056, 0.28, 0.026)),
        origin=Origin(xyz=(0.532, 0.0, 0.108)),
        material=armor_green,
        name="front_rim",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.034, length=0.18),
        origin=Origin(xyz=(0.24, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="recoil_boot",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.024, length=0.92),
        origin=Origin(xyz=(0.69, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_black,
        name="barrel",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.017, length=0.14),
        origin=Origin(xyz=(1.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_black,
        name="flash_hider",
    )
    weapon_cradle.visual(
        Box((0.20, 0.14, 0.22)),
        origin=Origin(xyz=(0.14, 0.26, 0.0)),
        material=armor_green,
        name="ammo_box",
    )
    weapon_cradle.visual(
        Box((0.10, 0.10, 0.12)),
        origin=Origin(xyz=(0.08, 0.18, 0.0)),
        material=dark_metal,
        name="ammo_bracket",
    )
    weapon_cradle.inertial = Inertial.from_geometry(
        Box((1.35, 0.55, 0.40)),
        mass=96.0,
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
    )

    service_lid = model.part("service_lid")
    service_lid.visual(
        Cylinder(radius=0.012, length=0.52),
        origin=Origin(xyz=(0.0, -0.004, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_gray,
        name="hinge_tube",
    )
    service_lid.visual(
        Box((0.58, 0.296, 0.018)),
        origin=Origin(xyz=(0.0, 0.148, 0.009)),
        material=armor_green,
        name="lid_panel",
    )
    service_lid.visual(
        Box((0.42, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, 0.090, -0.003)),
        material=armor_green,
        name="lid_rib_inner",
    )
    service_lid.visual(
        Box((0.42, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, 0.205, -0.003)),
        material=armor_green,
        name="lid_rib_outer",
    )
    service_lid.visual(
        Box((0.14, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, 0.170, 0.026)),
        material=dark_metal,
        name="lid_handle",
    )
    service_lid.inertial = Inertial.from_geometry(
        Box((0.58, 0.31, 0.06)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.15, 0.015)),
    )

    model.articulation(
        "roof_to_turret_yaw",
        ArticulationType.CONTINUOUS,
        parent=roof_base,
        child=turret_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=1.2),
    )

    model.articulation(
        "turret_to_weapon_pitch",
        ArticulationType.REVOLUTE,
        parent=turret_shell,
        child=weapon_cradle,
        origin=Origin(xyz=(0.35, 0.0, 0.29)),
        # Closed cradle geometry extends along +X from the side trunnions.
        # -Y makes positive q elevate the muzzle upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.0,
            lower=math.radians(-15.0),
            upper=math.radians(55.0),
        ),
    )

    model.articulation(
        "cradle_to_service_lid",
        ArticulationType.REVOLUTE,
        parent=weapon_cradle,
        child=service_lid,
        origin=Origin(xyz=(0.24, -0.148, 0.121)),
        # The closed lid panel extends along +Y from the hinge edge.
        # +X makes positive q swing the free edge upward.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=1.3,
            lower=0.0,
            upper=math.radians(115.0),
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

    roof_base = object_model.get_part("roof_base")
    turret_shell = object_model.get_part("turret_shell")
    weapon_cradle = object_model.get_part("weapon_cradle")
    service_lid = object_model.get_part("service_lid")

    yaw = object_model.get_articulation("roof_to_turret_yaw")
    pitch = object_model.get_articulation("turret_to_weapon_pitch")
    lid_hinge = object_model.get_articulation("cradle_to_service_lid")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.check(
        "yaw joint is vertical and continuous",
        yaw.articulation_type == ArticulationType.CONTINUOUS and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "pitch joint uses upward positive elevation",
        pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(pitch.axis) == (0.0, -1.0, 0.0)
        and pitch.motion_limits is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper > 0.5
        and pitch.motion_limits.lower < 0.0,
        details=f"type={pitch.articulation_type}, axis={pitch.axis}, limits={pitch.motion_limits}",
    )
    ctx.check(
        "service lid hinge opens from the closed top edge",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_hinge.axis) == (1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.5,
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )

    with ctx.pose({pitch: 0.0, lid_hinge: 0.0}):
        ctx.expect_gap(
            turret_shell,
            roof_base,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem="yaw_ring",
            negative_elem="bearing_race",
            name="yaw ring seats on the roof bearing",
        )
        ctx.expect_gap(
            service_lid,
            weapon_cradle,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem="lid_panel",
            negative_elem="front_rim",
            name="closed service lid sits flush on the housing rim",
        )
        ctx.expect_overlap(
            service_lid,
            weapon_cradle,
            axes="xy",
            elem_a="lid_panel",
            elem_b="bottom_pan",
            min_overlap=0.24,
            name="service lid spans the weapon housing",
        )

        rest_barrel_aabb = ctx.part_element_world_aabb(weapon_cradle, elem="barrel")
        rest_lid_aabb = ctx.part_element_world_aabb(service_lid, elem="lid_panel")

    pitch_upper = pitch.motion_limits.upper if pitch.motion_limits is not None else None
    if pitch_upper is not None and rest_barrel_aabb is not None:
        with ctx.pose({pitch: pitch_upper}):
            raised_barrel_aabb = ctx.part_element_world_aabb(weapon_cradle, elem="barrel")
        rest_barrel_center = _aabb_center(rest_barrel_aabb)
        raised_barrel_center = _aabb_center(raised_barrel_aabb)
        ctx.check(
            "weapon cradle elevates the barrel upward",
            rest_barrel_center is not None
            and raised_barrel_center is not None
            and raised_barrel_center[2] > rest_barrel_center[2] + 0.35
            and raised_barrel_center[0] < rest_barrel_center[0] - 0.10,
            details=f"rest={rest_barrel_center}, raised={raised_barrel_center}",
        )

    if rest_barrel_aabb is not None:
        with ctx.pose({yaw: math.pi / 2.0}):
            yawed_barrel_aabb = ctx.part_element_world_aabb(weapon_cradle, elem="barrel")
        rest_barrel_center = _aabb_center(rest_barrel_aabb)
        yawed_barrel_center = _aabb_center(yawed_barrel_aabb)
        ctx.check(
            "turret yaw rotates the cradle around the vertical axis",
            rest_barrel_center is not None
            and yawed_barrel_center is not None
            and rest_barrel_center[0] > 0.9
            and yawed_barrel_center[1] > 0.9
            and abs(yawed_barrel_center[2] - rest_barrel_center[2]) < 0.02,
            details=f"rest={rest_barrel_center}, yawed={yawed_barrel_center}",
        )

    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if lid_upper is not None and rest_lid_aabb is not None:
        with ctx.pose({lid_hinge: lid_upper}):
            open_lid_aabb = ctx.part_element_world_aabb(service_lid, elem="lid_panel")
        rest_lid_center = _aabb_center(rest_lid_aabb)
        open_lid_center = _aabb_center(open_lid_aabb)
        ctx.check(
            "service lid swings upward when opened",
            rest_lid_center is not None
            and open_lid_center is not None
            and open_lid_center[2] > rest_lid_center[2] + 0.12,
            details=f"closed={rest_lid_center}, open={open_lid_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
