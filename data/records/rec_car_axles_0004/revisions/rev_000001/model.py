from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    Mesh,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _add_wheel(
    model: ArticulatedObject,
    name: str,
    tire_mesh: Mesh,
    rim_mesh: Mesh,
    bead_mesh: Mesh,
    tire_material,
    rim_material,
):
    wheel = model.part(name)
    wheel.visual(
        tire_mesh,
        material=tire_material,
        name="tire_shell",
    )
    wheel.visual(
        rim_mesh,
        material=rim_material,
        name="rim_shell",
    )
    wheel.visual(
        bead_mesh,
        material=rim_material,
        name="bead_seat",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.46, length=0.16),
        mass=54.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    return wheel


def _build_shared_wheel_meshes() -> tuple[Mesh, Mesh, Mesh]:
    tire_outer = [
        (0.34, -0.08),
        (0.42, -0.08),
        (0.45, -0.06),
        (0.46, -0.03),
        (0.465, 0.0),
        (0.46, 0.03),
        (0.45, 0.06),
        (0.42, 0.08),
        (0.34, 0.08),
    ]
    tire_inner = [
        (0.28, -0.055),
        (0.31, -0.055),
        (0.335, -0.025),
        (0.34, 0.0),
        (0.335, 0.025),
        (0.31, 0.055),
        (0.28, 0.055),
    ]
    tire_geom = LatheGeometry.from_shell_profiles(
        tire_outer,
        tire_inner,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    tire_mesh = mesh_from_geometry(tire_geom, ASSETS.mesh_path("rear_axle_dual_tire.obj"))

    rim_outer = [
        (0.075, -0.08),
        (0.17, -0.08),
        (0.21, -0.07),
        (0.26, -0.055),
        (0.30, -0.045),
        (0.31, -0.025),
        (0.315, 0.0),
        (0.31, 0.025),
        (0.30, 0.045),
        (0.26, 0.055),
        (0.21, 0.07),
        (0.17, 0.08),
        (0.075, 0.08),
    ]
    rim_inner = [
        (0.065, -0.08),
        (0.065, 0.08),
    ]
    rim_geom = LatheGeometry.from_shell_profiles(
        rim_outer,
        rim_inner,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    rim_mesh = mesh_from_geometry(rim_geom, ASSETS.mesh_path("rear_axle_dual_rim.obj"))

    bead_outer = [
        (0.338, -0.06),
        (0.338, 0.06),
    ]
    bead_inner = [
        (0.305, -0.06),
        (0.305, 0.06),
    ]
    bead_geom = LatheGeometry.from_shell_profiles(
        bead_outer,
        bead_inner,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    bead_mesh = mesh_from_geometry(bead_geom, ASSETS.mesh_path("rear_axle_dual_bead_seat.obj"))
    return tire_mesh, rim_mesh, bead_mesh


def _build_differential_housing_mesh() -> Mesh:
    housing_profile = [
        (0.05, -0.25),
        (0.10, -0.24),
        (0.145, -0.20),
        (0.18, -0.14),
        (0.198, -0.07),
        (0.205, 0.0),
        (0.198, 0.07),
        (0.18, 0.14),
        (0.145, 0.20),
        (0.10, 0.24),
        (0.05, 0.25),
    ]
    housing_geom = LatheGeometry(housing_profile, segments=72).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(housing_geom, ASSETS.mesh_path("rear_axle_differential_housing.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="solid_rear_axle_dually", assets=ASSETS)

    housing_material = model.material("housing", rgba=(0.26, 0.26, 0.29, 1.0))
    steel_material = model.material("steel", rgba=(0.56, 0.57, 0.60, 1.0))
    machined_material = model.material("machined", rgba=(0.70, 0.71, 0.73, 1.0))
    wheel_material = model.material("wheel_steel", rgba=(0.67, 0.68, 0.70, 1.0))
    tire_material = model.material("tire", rgba=(0.08, 0.08, 0.09, 1.0))

    tire_mesh, rim_mesh, bead_mesh = _build_shared_wheel_meshes()
    differential_housing_mesh = _build_differential_housing_mesh()

    axle = model.part("axle_housing")
    axle.visual(
        differential_housing_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=housing_material,
        name="differential_housing",
    )
    axle.visual(
        Cylinder(radius=0.155, length=0.10),
        origin=Origin(xyz=(0.0, -0.115, -0.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_material,
        name="differential_cover",
    )
    axle.visual(
        Cylinder(radius=0.055, length=0.18),
        origin=Origin(xyz=(0.0, -0.25, -0.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_material,
        name="pinion_snout",
    )
    axle.visual(
        Cylinder(radius=0.032, length=0.75),
        origin=Origin(xyz=(-0.375, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_material,
        name="left_half_shaft",
    )
    axle.visual(
        Cylinder(radius=0.032, length=0.75),
        origin=Origin(xyz=(0.375, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_material,
        name="right_half_shaft",
    )
    axle.visual(
        Cylinder(radius=0.08, length=0.56),
        origin=Origin(xyz=(-0.46, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_material,
        name="left_axle_tube",
    )
    axle.visual(
        Cylinder(radius=0.08, length=0.56),
        origin=Origin(xyz=(0.46, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_material,
        name="right_axle_tube",
    )
    axle.visual(
        Cylinder(radius=0.055, length=0.39),
        origin=Origin(xyz=(-0.935, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_material,
        name="left_spindle",
    )
    axle.visual(
        Cylinder(radius=0.055, length=0.39),
        origin=Origin(xyz=(0.935, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_material,
        name="right_spindle",
    )
    axle.visual(
        Cylinder(radius=0.17, length=0.02),
        origin=Origin(xyz=(-0.75, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_material,
        name="left_inner_flange",
    )
    axle.visual(
        Cylinder(radius=0.17, length=0.02),
        origin=Origin(xyz=(0.75, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_material,
        name="right_inner_flange",
    )
    axle.visual(
        Cylinder(radius=0.17, length=0.03),
        origin=Origin(xyz=(-0.935, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_material,
        name="left_dual_spacer",
    )
    axle.visual(
        Cylinder(radius=0.17, length=0.03),
        origin=Origin(xyz=(0.935, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_material,
        name="right_dual_spacer",
    )
    axle.visual(
        Cylinder(radius=0.16, length=0.02),
        origin=Origin(xyz=(-1.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_material,
        name="left_outer_clamp",
    )
    axle.visual(
        Cylinder(radius=0.16, length=0.02),
        origin=Origin(xyz=(1.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_material,
        name="right_outer_clamp",
    )
    axle.inertial = Inertial.from_geometry(
        Box((2.30, 0.44, 0.92)),
        mass=295.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_inner_wheel = _add_wheel(
        model,
        "left_inner_wheel",
        tire_mesh,
        rim_mesh,
        bead_mesh,
        tire_material,
        wheel_material,
    )
    left_outer_wheel = _add_wheel(
        model,
        "left_outer_wheel",
        tire_mesh,
        rim_mesh,
        bead_mesh,
        tire_material,
        wheel_material,
    )
    right_inner_wheel = _add_wheel(
        model,
        "right_inner_wheel",
        tire_mesh,
        rim_mesh,
        bead_mesh,
        tire_material,
        wheel_material,
    )
    right_outer_wheel = _add_wheel(
        model,
        "right_outer_wheel",
        tire_mesh,
        rim_mesh,
        bead_mesh,
        tire_material,
        wheel_material,
    )

    wheel_limits = MotionLimits(effort=350.0, velocity=32.0)
    for art_name, child, x_pos in (
        ("axle_to_left_inner_wheel", left_inner_wheel, -0.84),
        ("axle_to_left_outer_wheel", left_outer_wheel, -1.03),
        ("axle_to_right_inner_wheel", right_inner_wheel, 0.84),
        ("axle_to_right_outer_wheel", right_outer_wheel, 1.03),
    ):
        model.articulation(
            art_name,
            ArticulationType.CONTINUOUS,
            parent=axle,
            child=child,
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=wheel_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    axle = object_model.get_part("axle_housing")
    left_inner_wheel = object_model.get_part("left_inner_wheel")
    left_outer_wheel = object_model.get_part("left_outer_wheel")
    right_inner_wheel = object_model.get_part("right_inner_wheel")
    right_outer_wheel = object_model.get_part("right_outer_wheel")

    left_inner_spin = object_model.get_articulation("axle_to_left_inner_wheel")
    left_outer_spin = object_model.get_articulation("axle_to_left_outer_wheel")
    right_inner_spin = object_model.get_articulation("axle_to_right_inner_wheel")
    right_outer_spin = object_model.get_articulation("axle_to_right_outer_wheel")

    differential_housing = axle.get_visual("differential_housing")
    left_inner_flange = axle.get_visual("left_inner_flange")
    right_inner_flange = axle.get_visual("right_inner_flange")
    left_dual_spacer = axle.get_visual("left_dual_spacer")
    right_dual_spacer = axle.get_visual("right_dual_spacer")
    left_outer_clamp = axle.get_visual("left_outer_clamp")
    right_outer_clamp = axle.get_visual("right_outer_clamp")

    left_inner_rim = left_inner_wheel.get_visual("rim_shell")
    left_outer_rim = left_outer_wheel.get_visual("rim_shell")
    right_inner_rim = right_inner_wheel.get_visual("rim_shell")
    right_outer_rim = right_outer_wheel.get_visual("rim_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=8)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "wheel_joint_axes",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in (left_inner_spin, left_outer_spin, right_inner_spin, right_outer_spin)),
        "All wheel spin joints should rotate about the axle centerline.",
    )
    ctx.check(
        "wheel_joint_types",
        all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in (left_inner_spin, left_outer_spin, right_inner_spin, right_outer_spin)),
        "Truck wheels should use continuous spin joints.",
    )

    ctx.expect_contact(left_inner_wheel, axle, elem_a=left_inner_rim, elem_b=left_inner_flange)
    ctx.expect_contact(left_inner_wheel, axle, elem_a=left_inner_rim, elem_b=left_dual_spacer)
    ctx.expect_contact(left_outer_wheel, axle, elem_a=left_outer_rim, elem_b=left_dual_spacer)
    ctx.expect_contact(left_outer_wheel, axle, elem_a=left_outer_rim, elem_b=left_outer_clamp)

    ctx.expect_contact(right_inner_wheel, axle, elem_a=right_inner_rim, elem_b=right_inner_flange)
    ctx.expect_contact(right_inner_wheel, axle, elem_a=right_inner_rim, elem_b=right_dual_spacer)
    ctx.expect_contact(right_outer_wheel, axle, elem_a=right_outer_rim, elem_b=right_dual_spacer)
    ctx.expect_contact(right_outer_wheel, axle, elem_a=right_outer_rim, elem_b=right_outer_clamp)

    ctx.expect_gap(
        axle,
        left_inner_wheel,
        axis="x",
        min_gap=0.50,
        positive_elem=differential_housing,
        negative_elem=left_inner_rim,
        name="left_wheels_outboard_of_differential",
    )
    ctx.expect_gap(
        right_inner_wheel,
        axle,
        axis="x",
        min_gap=0.50,
        positive_elem=right_inner_rim,
        negative_elem=differential_housing,
        name="right_wheels_outboard_of_differential",
    )
    ctx.expect_gap(
        left_inner_wheel,
        left_outer_wheel,
        axis="x",
        min_gap=0.029,
        max_gap=0.031,
        name="left_dual_wheel_gap",
    )
    ctx.expect_gap(
        right_outer_wheel,
        right_inner_wheel,
        axis="x",
        min_gap=0.029,
        max_gap=0.031,
        name="right_dual_wheel_gap",
    )
    ctx.expect_origin_distance(
        left_inner_wheel,
        left_outer_wheel,
        axes="yz",
        max_dist=0.001,
    )
    ctx.expect_origin_distance(
        right_inner_wheel,
        right_outer_wheel,
        axes="yz",
        max_dist=0.001,
    )
    ctx.expect_overlap(axle, left_inner_wheel, axes="yz", min_overlap=0.10)
    ctx.expect_overlap(axle, right_inner_wheel, axes="yz", min_overlap=0.10)

    with ctx.pose(
        {
            left_inner_spin: 3.0,
            left_outer_spin: -2.4,
            right_inner_spin: 2.5,
            right_outer_spin: -3.5,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="posed_no_overlap")
        ctx.fail_if_isolated_parts(name="posed_no_floating")
        ctx.expect_contact(left_inner_wheel, axle, elem_a=left_inner_rim, elem_b=left_inner_flange)
        ctx.expect_contact(left_outer_wheel, axle, elem_a=left_outer_rim, elem_b=left_dual_spacer)
        ctx.expect_contact(right_inner_wheel, axle, elem_a=right_inner_rim, elem_b=right_inner_flange)
        ctx.expect_contact(right_outer_wheel, axle, elem_a=right_outer_rim, elem_b=right_dual_spacer)
        ctx.expect_origin_distance(left_inner_wheel, left_outer_wheel, axes="yz", max_dist=0.001)
        ctx.expect_origin_distance(right_inner_wheel, right_outer_wheel, axes="yz", max_dist=0.001)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
