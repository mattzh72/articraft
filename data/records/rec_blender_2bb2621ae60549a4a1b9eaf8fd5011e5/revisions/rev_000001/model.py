from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_cup_shell():
    outer_profile = [
        (0.066, 0.000),
        (0.066, 0.018),
        (0.058, 0.030),
        (0.058, 0.192),
        (0.060, 0.215),
    ]
    inner_profile = [
        (0.016, 0.000),
        (0.016, 0.012),
        (0.050, 0.020),
        (0.052, 0.190),
        (0.053, 0.209),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "chamber_cup_shell",
    )


def _build_locking_collar():
    outer_profile = [
        (0.074, 0.000),
        (0.074, 0.010),
        (0.072, 0.018),
        (0.070, 0.026),
    ]
    inner_profile = [
        (0.067, 0.000),
        (0.067, 0.012),
        (0.065, 0.020),
        (0.063, 0.026),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        ),
        "locking_collar",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def _aabb_dims(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple(max_corner[i] - min_corner[i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_homogenizer_blender")

    housing_white = model.material("housing_white", rgba=(0.92, 0.93, 0.91, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.16, 0.18, 0.20, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.12, 0.12, 0.13, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.200, 0.240, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=housing_white,
        name="base_plinth",
    )
    housing.visual(
        Box((0.164, 0.198, 0.300)),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=housing_white,
        name="tower_body",
    )
    housing.visual(
        Box((0.132, 0.132, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material=housing_white,
        name="top_deck",
    )
    housing.visual(
        Box((0.118, 0.118, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=housing_white,
        name="upper_neck",
    )
    housing.visual(
        _build_locking_collar(),
        origin=Origin(xyz=(0.0, 0.0, 0.404)),
        material=black_polymer,
        name="locking_collar",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.427)),
        material=dark_panel,
        name="drive_stud",
    )
    housing.visual(
        Box((0.010, 0.096, 0.150)),
        origin=Origin(xyz=(0.087, 0.0, 0.208)),
        material=dark_panel,
        name="control_panel",
    )
    housing.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.101, 0.0, 0.182), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_polymer,
        name="speed_knob",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.200, 0.240, 0.440)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
    )

    chamber_cup = model.part("chamber_cup")
    chamber_cup.visual(
        _build_cup_shell(),
        material=stainless,
        name="cup_shell",
    )
    chamber_cup.visual(
        Box((0.028, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.071, 0.014)),
        material=stainless,
        name="lock_lug_right",
    )
    chamber_cup.visual(
        Box((0.028, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.071, 0.014)),
        material=stainless,
        name="lock_lug_left",
    )
    chamber_cup.visual(
        Box((0.012, 0.018, 0.020)),
        origin=Origin(xyz=(-0.071, 0.032, 0.212)),
        material=black_polymer,
        name="hinge_block_right",
    )
    chamber_cup.visual(
        Box((0.012, 0.018, 0.020)),
        origin=Origin(xyz=(-0.071, -0.032, 0.212)),
        material=black_polymer,
        name="hinge_block_left",
    )
    chamber_cup.visual(
        Box((0.020, 0.014, 0.016)),
        origin=Origin(xyz=(-0.060, 0.032, 0.195)),
        material=black_polymer,
        name="hinge_strut_right",
    )
    chamber_cup.visual(
        Box((0.020, 0.014, 0.016)),
        origin=Origin(xyz=(-0.060, -0.032, 0.195)),
        material=black_polymer,
        name="hinge_strut_left",
    )
    chamber_cup.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(
            xyz=(-0.066, 0.030, 0.221),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black_polymer,
        name="hinge_ear_right",
    )
    chamber_cup.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(
            xyz=(-0.066, -0.030, 0.221),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black_polymer,
        name="hinge_ear_left",
    )
    chamber_cup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.066, length=0.215),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
    )

    chamber_lid = model.part("chamber_lid")
    chamber_lid.visual(
        Cylinder(radius=0.063, length=0.012),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=black_polymer,
        name="lid_cap",
    )
    chamber_lid.visual(
        Cylinder(radius=0.051, length=0.010),
        origin=Origin(xyz=(0.066, 0.0, -0.011)),
        material=black_polymer,
        name="lid_plug",
    )
    chamber_lid.visual(
        Box((0.028, 0.026, 0.008)),
        origin=Origin(xyz=(0.120, 0.0, 0.001)),
        material=black_polymer,
        name="lid_pull_tab",
    )
    chamber_lid.visual(
        Box((0.020, 0.046, 0.010)),
        origin=Origin(xyz=(0.016, 0.0, 0.000)),
        material=black_polymer,
        name="lid_bridge",
    )
    chamber_lid.visual(
        Cylinder(radius=0.0052, length=0.036),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black_polymer,
        name="lid_hinge_barrel",
    )
    chamber_lid.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.072, 0.0, 0.013)),
        material=black_polymer,
        name="lid_grip_boss",
    )
    chamber_lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.063, length=0.028),
        mass=0.16,
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
    )

    blade_drive = model.part("blade_drive")
    blade_drive.visual(
        Cylinder(radius=0.007, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=stainless,
        name="blade_shaft",
    )
    blade_drive.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=stainless,
        name="blade_hub",
    )
    blade_drive.visual(
        Box((0.052, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.029), rpy=(0.22, 0.0, 0.0)),
        material=stainless,
        name="blade_wing_a",
    )
    blade_drive.visual(
        Box((0.038, 0.010, 0.004)),
        origin=Origin(
            xyz=(0.0, 0.0, 0.021),
            rpy=(-0.20, 0.0, math.pi / 2.0),
        ),
        material=stainless,
        name="blade_wing_b",
    )
    blade_drive.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.050),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "housing_to_cup_lock",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=chamber_cup,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(24.0),
        ),
    )
    model.articulation(
        "cup_to_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=chamber_cup,
        child=chamber_lid,
        origin=Origin(xyz=(-0.066, 0.0, 0.221)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )
    model.articulation(
        "housing_to_blade_drive",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade_drive,
        origin=Origin(xyz=(0.0, 0.0, 0.434)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
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

    housing = object_model.get_part("housing")
    chamber_cup = object_model.get_part("chamber_cup")
    chamber_lid = object_model.get_part("chamber_lid")
    blade_drive = object_model.get_part("blade_drive")

    cup_lock = object_model.get_articulation("housing_to_cup_lock")
    lid_hinge = object_model.get_articulation("cup_to_lid_hinge")
    blade_spin = object_model.get_articulation("housing_to_blade_drive")

    ctx.check(
        "cup bayonet twist is a vertical revolute joint",
        cup_lock.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in cup_lock.axis) == (0.0, 0.0, 1.0),
        details=f"type={cup_lock.articulation_type}, axis={cup_lock.axis}",
    )
    ctx.check(
        "lid hinge opens upward from the rear edge",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in lid_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}",
    )
    ctx.check(
        "blade drive spins continuously about the cup axis",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in blade_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={blade_spin.articulation_type}, axis={blade_spin.axis}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            chamber_lid,
            chamber_cup,
            axis="z",
            positive_elem="lid_cap",
            negative_elem="cup_shell",
            max_gap=0.008,
            max_penetration=0.0,
            name="lid cap seats on the cup rim",
        )
        ctx.expect_overlap(
            chamber_lid,
            chamber_cup,
            axes="xy",
            elem_a="lid_cap",
            elem_b="cup_shell",
            min_overlap=0.100,
            name="lid covers the chamber opening",
        )

    ctx.expect_gap(
        chamber_cup,
        housing,
        axis="z",
        positive_elem="cup_shell",
        negative_elem="top_deck",
        max_gap=0.001,
        max_penetration=0.0,
        name="cup seats directly on the housing deck",
    )
    ctx.expect_within(
        blade_drive,
        chamber_cup,
        axes="xy",
        inner_elem="blade_wing_a",
        outer_elem="cup_shell",
        margin=0.004,
        name="blade stays centered within the chamber cup",
    )
    ctx.expect_overlap(
        blade_drive,
        chamber_cup,
        axes="z",
        elem_a="blade_shaft",
        elem_b="cup_shell",
        min_overlap=0.030,
        name="blade drive rises into the chamber volume",
    )

    lid_open = lid_hinge.motion_limits.upper or math.radians(90.0)
    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_aabb = ctx.part_world_aabb(chamber_lid)
    with ctx.pose({lid_hinge: lid_open}):
        open_lid_aabb = ctx.part_world_aabb(chamber_lid)
    ctx.check(
        "lid swings upward when opened",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.090,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    cup_twist = cup_lock.motion_limits.upper or math.radians(20.0)
    with ctx.pose({cup_lock: 0.0}):
        rest_lug_aabb = ctx.part_element_world_aabb(chamber_cup, elem="lock_lug_right")
    with ctx.pose({cup_lock: cup_twist}):
        twisted_lug_aabb = ctx.part_element_world_aabb(chamber_cup, elem="lock_lug_right")
    rest_lug_center = _aabb_center(rest_lug_aabb)
    twisted_lug_center = _aabb_center(twisted_lug_aabb)
    ctx.check(
        "bayonet twist moves the locking lug around the cup axis",
        rest_lug_center is not None
        and twisted_lug_center is not None
        and math.dist(rest_lug_center[:2], twisted_lug_center[:2]) > 0.018
        and abs(math.hypot(rest_lug_center[0], rest_lug_center[1]) - math.hypot(twisted_lug_center[0], twisted_lug_center[1])) < 0.004,
        details=f"rest={rest_lug_center}, twisted={twisted_lug_center}",
    )

    with ctx.pose({blade_spin: 0.0}):
        wing_rest_aabb = ctx.part_element_world_aabb(blade_drive, elem="blade_wing_a")
    with ctx.pose({blade_spin: math.pi / 2.0}):
        wing_spin_aabb = ctx.part_element_world_aabb(blade_drive, elem="blade_wing_a")
    wing_rest_dims = _aabb_dims(wing_rest_aabb)
    wing_spin_dims = _aabb_dims(wing_spin_aabb)
    ctx.check(
        "blade wing rotates about the vertical drive axis",
        wing_rest_dims is not None
        and wing_spin_dims is not None
        and wing_rest_dims[0] > wing_rest_dims[1] * 2.0
        and wing_spin_dims[1] > wing_spin_dims[0] * 2.0,
        details=f"rest={wing_rest_dims}, spun={wing_spin_dims}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
