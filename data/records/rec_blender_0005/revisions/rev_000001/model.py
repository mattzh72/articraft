from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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


ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_cup_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.037, 0.000),
            (0.039, 0.014),
            (0.045, 0.055),
            (0.051, 0.120),
            (0.053, 0.190),
            (0.051, 0.225),
            (0.047, 0.242),
            (0.044, 0.258),
        ],
        [
            (0.033, 0.004),
            (0.034, 0.014),
            (0.041, 0.055),
            (0.047, 0.120),
            (0.049, 0.188),
            (0.046, 0.223),
            (0.042, 0.236),
            (0.040, 0.254),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="personal_blender", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber_gray = model.material("rubber_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.83, 0.89, 0.94, 0.32))
    collar_gray = model.material("collar_gray", rgba=(0.58, 0.61, 0.65, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.76, 0.78, 0.81, 1.0))

    motor_base = model.part("motor_base")
    motor_base.visual(
        Cylinder(radius=0.061, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rubber_gray,
        name="foot_ring",
    )
    motor_base.visual(
        Cylinder(radius=0.057, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=matte_black,
        name="motor_housing",
    )
    motor_base.visual(
        Cylinder(radius=0.044, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=dark_gray,
        name="upper_shoulder",
    )
    motor_base.visual(
        Cylinder(radius=0.035, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        material=collar_gray,
        name="landing_pad",
    )
    motor_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.061, length=0.126),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
    )

    blending_cup = model.part("blending_cup")
    blending_cup.visual(
        _save_mesh("cup_shell.obj", _build_cup_shell()),
        material=smoke_clear,
        name="cup_shell",
    )
    blending_cup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.053, length=0.256),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=collar_gray,
        name="seal_flange",
    )
    blade_assembly.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_gray,
        name="blade_carrier",
    )
    blade_assembly.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=blade_steel,
        name="blade_post",
    )
    blade_assembly.visual(
        Box((0.040, 0.006, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.024), rpy=(0.0, 0.24, 0.0)),
        material=blade_steel,
        name="blade_long",
    )
    blade_assembly.visual(
        Box((0.006, 0.034, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.024), rpy=(-0.24, 0.0, 0.0)),
        material=blade_steel,
        name="blade_cross",
    )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.028),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "cup_lock",
        ArticulationType.REVOLUTE,
        parent=motor_base,
        child=blending_cup,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.REVOLUTE,
        parent=blending_cup,
        child=blade_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=80.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    motor_base = object_model.get_part("motor_base")
    blending_cup = object_model.get_part("blending_cup")
    blade_assembly = object_model.get_part("blade_assembly")
    cup_lock = object_model.get_articulation("cup_lock")
    blade_spin = object_model.get_articulation("blade_spin")
    landing_pad = motor_base.get_visual("landing_pad")
    cup_shell = blending_cup.get_visual("cup_shell")
    seal_flange = blade_assembly.get_visual("seal_flange")
    blade_carrier = blade_assembly.get_visual("blade_carrier")
    blade_long = blade_assembly.get_visual("blade_long")
    blade_cross = blade_assembly.get_visual("blade_cross")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=16)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "cup_lock_axis_is_vertical",
        tuple(cup_lock.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical cup-lock axis, got {cup_lock.axis!r}",
    )
    ctx.check(
        "blade_spin_axis_is_vertical",
        tuple(blade_spin.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical blade axis, got {blade_spin.axis!r}",
    )

    ctx.expect_contact(blade_assembly, motor_base, elem_a=seal_flange, elem_b=landing_pad)
    ctx.expect_gap(
        blade_assembly,
        motor_base,
        axis="z",
        max_gap=0.0001,
        max_penetration=0.0,
        positive_elem=seal_flange,
        negative_elem=landing_pad,
    )
    ctx.expect_contact(blending_cup, blade_assembly)
    ctx.expect_gap(
        blending_cup,
        motor_base,
        axis="z",
        max_gap=0.0001,
        max_penetration=0.0,
        positive_elem=cup_shell,
        negative_elem=landing_pad,
    )
    ctx.expect_overlap(blending_cup, motor_base, axes="xy", min_overlap=0.10)
    ctx.expect_origin_distance(blending_cup, motor_base, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(blade_assembly, blending_cup, axes="xy", max_dist=0.001)
    ctx.expect_within(
        blade_assembly,
        blending_cup,
        axes="xy",
        inner_elem=blade_long,
        outer_elem=cup_shell,
    )
    ctx.expect_within(
        blade_assembly,
        blending_cup,
        axes="xy",
        inner_elem=blade_cross,
        outer_elem=cup_shell,
    )

    base_aabb = ctx.part_world_aabb(motor_base)
    cup_aabb = ctx.part_world_aabb(blending_cup)
    blade_aabb = ctx.part_world_aabb(blade_assembly)
    assert base_aabb is not None
    assert cup_aabb is not None
    assert blade_aabb is not None

    base_height = base_aabb[1][2] - base_aabb[0][2]
    cup_height = cup_aabb[1][2] - cup_aabb[0][2]
    cup_diameter = max(cup_aabb[1][0] - cup_aabb[0][0], cup_aabb[1][1] - cup_aabb[0][1])
    base_diameter = max(base_aabb[1][0] - base_aabb[0][0], base_aabb[1][1] - base_aabb[0][1])
    blade_height = blade_aabb[1][2] - blade_aabb[0][2]

    ctx.check(
        "cup_is_taller_than_motor_base",
        cup_height > base_height * 1.9,
        details=f"Cup height {cup_height:.4f} should be much taller than base height {base_height:.4f}",
    )
    ctx.check(
        "cup_matches_personal_blender_diameter",
        0.09 <= cup_diameter <= 0.12 and 0.10 <= base_diameter <= 0.13,
        details=(
            f"Unexpected diameters: cup={cup_diameter:.4f} m, "
            f"base={base_diameter:.4f} m"
        ),
    )
    ctx.check(
        "blade_stack_is_compact",
        0.02 <= blade_height <= 0.03,
        details=f"Blade stack height {blade_height:.4f} m should stay compact at the cup neck",
    )

    with ctx.pose({cup_lock: math.radians(35.0), blade_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="posed_no_overlap")
        ctx.fail_if_isolated_parts(name="posed_no_floating")
        ctx.expect_origin_distance(blade_assembly, blending_cup, axes="xy", max_dist=0.001)
        ctx.expect_contact(blade_assembly, motor_base, elem_a=seal_flange, elem_b=landing_pad)
        ctx.expect_gap(
            blade_assembly,
            motor_base,
            axis="z",
            max_gap=0.0001,
            max_penetration=0.0,
            positive_elem=seal_flange,
            negative_elem=landing_pad,
        )
        ctx.expect_gap(
            blending_cup,
            motor_base,
            axis="z",
            max_gap=0.0001,
            max_penetration=0.0,
            positive_elem=cup_shell,
            negative_elem=landing_pad,
        )
        ctx.expect_within(
            blade_assembly,
            blending_cup,
            axes="xy",
            inner_elem=blade_long,
            outer_elem=cup_shell,
        )
        ctx.expect_within(
            blade_assembly,
            blending_cup,
            axes="xy",
            inner_elem=blade_cross,
            outer_elem=cup_shell,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
