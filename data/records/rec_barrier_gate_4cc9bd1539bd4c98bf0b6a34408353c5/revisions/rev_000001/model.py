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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_flat_street_bollard")

    zinc_steel = model.material("zinc_steel", rgba=(0.52, 0.54, 0.56, 1.0))
    dark_galvanized = model.material("dark_galvanized", rgba=(0.30, 0.32, 0.35, 1.0))
    worn_black = model.material("worn_black", rgba=(0.14, 0.14, 0.15, 1.0))
    reflector_yellow = model.material("reflector_yellow", rgba=(0.92, 0.78, 0.20, 1.0))

    receiver_base = model.part("receiver_base")

    receiver_shell = LatheGeometry.from_shell_profiles(
        [
            (0.078, -0.240),
            (0.078, -0.018),
            (0.090, -0.006),
            (0.125, 0.000),
            (0.125, 0.010),
        ],
        [
            (0.060, -0.225),
            (0.060, -0.014),
            (0.065, -0.002),
            (0.072, 0.004),
        ],
        segments=56,
    )
    receiver_base.visual(
        mesh_from_geometry(receiver_shell, "receiver_socket"),
        material=dark_galvanized,
        name="receiver_socket",
    )
    receiver_base.visual(
        Box((0.090, 0.120, 0.030)),
        origin=Origin(xyz=(-0.082, 0.000, 0.015)),
        material=dark_galvanized,
        name="hinge_pedestal",
    )
    receiver_base.visual(
        Box((0.024, 0.028, 0.070)),
        origin=Origin(xyz=(-0.067, -0.035, 0.065)),
        material=dark_galvanized,
        name="hinge_cheek_neg_y",
    )
    receiver_base.visual(
        Box((0.024, 0.028, 0.070)),
        origin=Origin(xyz=(-0.067, 0.035, 0.065)),
        material=dark_galvanized,
        name="hinge_cheek_pos_y",
    )
    receiver_base.visual(
        Box((0.024, 0.012, 0.070)),
        origin=Origin(xyz=(-0.067, 0.051, 0.065)),
        material=dark_galvanized,
        name="lever_mount_cheek",
    )
    receiver_base.inertial = Inertial.from_geometry(
        Box((0.250, 0.180, 0.350)),
        mass=32.0,
        origin=Origin(xyz=(0.000, 0.000, -0.115)),
    )

    bollard_post = model.part("bollard_post")
    bollard_post.visual(
        Cylinder(radius=0.015, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
        name="hinge_barrel",
    )
    bollard_post.visual(
        Box((0.037, 0.032, 0.040)),
        origin=Origin(xyz=(0.026, 0.000, 0.020)),
        material=zinc_steel,
        name="hinge_link",
    )
    bollard_post.visual(
        Box((0.075, 0.082, 0.050)),
        origin=Origin(xyz=(0.082, 0.000, 0.025)),
        material=zinc_steel,
        name="base_shoe",
    )
    bollard_post.visual(
        Cylinder(radius=0.051, length=0.820),
        origin=Origin(xyz=(0.067, 0.000, 0.430)),
        material=zinc_steel,
        name="post_shaft",
    )
    bollard_post.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.067, 0.000, 0.860)),
        material=zinc_steel,
        name="top_cap",
    )
    bollard_post.visual(
        Cylinder(radius=0.053, length=0.120),
        origin=Origin(xyz=(0.067, 0.000, 0.660)),
        material=reflector_yellow,
        name="reflective_band",
    )
    bollard_post.visual(
        Box((0.014, 0.024, 0.028)),
        origin=Origin(xyz=(0.034, 0.043, 0.036)),
        material=zinc_steel,
        name="post_latch_tab",
    )
    bollard_post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.920),
        mass=17.5,
        origin=Origin(xyz=(0.067, 0.000, 0.460)),
    )

    locking_lever = model.part("locking_lever")
    locking_lever.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.000, 0.006, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_black,
        name="lever_pivot_barrel",
    )
    locking_lever.visual(
        Box((0.020, 0.010, 0.014)),
        origin=Origin(xyz=(0.010, 0.005, 0.010)),
        material=worn_black,
        name="lever_link",
    )
    locking_lever.visual(
        Box((0.028, 0.010, 0.034)),
        origin=Origin(xyz=(0.030, 0.005, 0.018)),
        material=worn_black,
        name="latch_cam",
    )
    locking_lever.visual(
        Box((0.012, 0.010, 0.105)),
        origin=Origin(xyz=(0.046, 0.005, 0.070), rpy=(0.0, 0.0, -0.18)),
        material=worn_black,
        name="lever_handle",
    )
    locking_lever.inertial = Inertial.from_geometry(
        Box((0.055, 0.014, 0.120)),
        mass=0.35,
        origin=Origin(xyz=(0.022, 0.005, 0.050)),
    )

    model.articulation(
        "base_to_post",
        ArticulationType.REVOLUTE,
        parent=receiver_base,
        child=bollard_post,
        origin=Origin(xyz=(-0.067, 0.000, 0.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.3,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "base_to_locking_lever",
        ArticulationType.REVOLUTE,
        parent=receiver_base,
        child=locking_lever,
        origin=Origin(xyz=(-0.067, 0.057, 0.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(42.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    receiver_base = object_model.get_part("receiver_base")
    bollard_post = object_model.get_part("bollard_post")
    locking_lever = object_model.get_part("locking_lever")
    post_hinge = object_model.get_articulation("base_to_post")
    lever_hinge = object_model.get_articulation("base_to_locking_lever")

    with ctx.pose({post_hinge: 0.0, lever_hinge: 0.0}):
        ctx.expect_contact(
            locking_lever,
            receiver_base,
            elem_a="lever_pivot_barrel",
            elem_b="lever_mount_cheek",
            name="locking lever remains carried by the base cheek at its pivot",
        )
        ctx.expect_overlap(
            bollard_post,
            receiver_base,
            axes="xy",
            min_overlap=0.070,
            elem_a="post_shaft",
            elem_b="receiver_socket",
            name="upright post remains over the receiver opening",
        )
        ctx.expect_overlap(
            locking_lever,
            bollard_post,
            axes="xz",
            min_overlap=0.010,
            elem_a="latch_cam",
            elem_b="post_latch_tab",
            name="locking cam aligns with the post latch tab",
        )
        ctx.expect_gap(
            locking_lever,
            bollard_post,
            axis="y",
            min_gap=0.001,
            max_gap=0.010,
            positive_elem="latch_cam",
            negative_elem="post_latch_tab",
            name="locking cam sits just outside the post latch tab",
        )

        upright_shaft = ctx.part_element_world_aabb(bollard_post, elem="post_shaft")
        upright_handle = ctx.part_element_world_aabb(locking_lever, elem="lever_handle")

    with ctx.pose({post_hinge: post_hinge.motion_limits.upper}):
        folded_shaft = ctx.part_element_world_aabb(bollard_post, elem="post_shaft")

    ctx.check(
        "post folds flat from upright toward +x",
        upright_shaft is not None
        and folded_shaft is not None
        and upright_shaft[1][2] > 0.82
        and folded_shaft[1][0] > 0.74
        and folded_shaft[1][2] < 0.18,
        details=f"upright_shaft={upright_shaft}, folded_shaft={folded_shaft}",
    )

    with ctx.pose({lever_hinge: lever_hinge.motion_limits.upper}):
        released_handle = ctx.part_element_world_aabb(locking_lever, elem="lever_handle")

    ctx.check(
        "locking lever swings outward on its short revolute",
        upright_handle is not None
        and released_handle is not None
        and released_handle[1][0] > upright_handle[1][0] + 0.02
        and released_handle[1][2] < upright_handle[1][2] - 0.015,
        details=f"upright_handle={upright_handle}, released_handle={released_handle}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
