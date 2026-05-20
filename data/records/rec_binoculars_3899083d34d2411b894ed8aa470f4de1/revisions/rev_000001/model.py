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


BARREL_CENTER_X = 0.024
BARREL_LENGTH = 0.056
FOCUS_RING_CENTER_Y = -0.017


def _barrel_shell_mesh():
    outer_profile = [
        (0.0095, -0.0280),
        (0.0115, -0.0260),
        (0.0144, -0.0230),
        (0.0158, -0.0180),
        (0.0158, -0.0040),
        (0.0150, 0.0160),
        (0.0160, 0.0240),
        (0.0166, 0.0280),
    ]
    inner_profile = [
        (0.0045, -0.0240),
        (0.0082, -0.0200),
        (0.0108, -0.0100),
        (0.0113, 0.0160),
        (0.0118, 0.0240),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "opera_glasses_barrel_shell",
    )


def _focus_ring_mesh():
    outer_profile = [
        (0.0172, -0.0060),
        (0.0179, -0.0042),
        (0.0184, 0.0000),
        (0.0179, 0.0042),
        (0.0172, 0.0060),
    ]
    inner_profile = [
        (0.0164, -0.0062),
        (0.0164, 0.0062),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "opera_glasses_focus_ring",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="opera_glasses")

    black_enamel = model.material("black_enamel", rgba=(0.10, 0.08, 0.07, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.76, 0.64, 0.34, 1.0))
    dark_brass = model.material("dark_brass", rgba=(0.51, 0.40, 0.18, 1.0))

    barrel_shell = _barrel_shell_mesh()
    focus_ring_shell = _focus_ring_mesh()

    left_half = model.part("left_half")
    left_half.visual(
        barrel_shell,
        origin=Origin(xyz=(-BARREL_CENTER_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_enamel,
        name="left_barrel_shell",
    )
    left_half.visual(
        Cylinder(radius=0.0042, length=0.018),
        origin=Origin(xyz=(-0.012, -0.008, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="left_bridge_arm",
    )
    left_half.visual(
        Box((0.004, 0.010, 0.026)),
        origin=Origin(xyz=(-0.006, -0.003, 0.0)),
        material=dark_brass,
        name="left_hinge_cheek",
    )
    left_half.visual(
        Cylinder(radius=0.0049, length=0.0072),
        origin=Origin(xyz=(0.0, 0.0, 0.0081)),
        material=dark_brass,
        name="left_hinge_knuckle_upper",
    )
    left_half.visual(
        Cylinder(radius=0.0049, length=0.0072),
        origin=Origin(xyz=(0.0, 0.0, -0.0081)),
        material=dark_brass,
        name="left_hinge_knuckle_lower",
    )
    left_half.inertial = Inertial.from_geometry(
        Box((0.050, 0.060, 0.028)),
        mass=0.16,
        origin=Origin(xyz=(-0.018, 0.0, 0.0)),
    )

    right_half = model.part("right_half")
    right_half.visual(
        barrel_shell,
        origin=Origin(xyz=(BARREL_CENTER_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_enamel,
        name="right_barrel_shell",
    )
    right_half.visual(
        Cylinder(radius=0.0042, length=0.018),
        origin=Origin(xyz=(0.012, -0.008, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="right_bridge_arm",
    )
    right_half.visual(
        Box((0.004, 0.010, 0.018)),
        origin=Origin(xyz=(0.006, -0.003, 0.0)),
        material=dark_brass,
        name="right_hinge_cheek",
    )
    right_half.visual(
        Cylinder(radius=0.0044, length=0.0080),
        origin=Origin(),
        material=dark_brass,
        name="right_hinge_knuckle",
    )
    right_half.inertial = Inertial.from_geometry(
        Box((0.050, 0.060, 0.028)),
        mass=0.16,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    left_focus_ring = model.part("left_focus_ring")
    left_focus_ring.visual(
        focus_ring_shell,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="left_focus_shell",
    )
    left_focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0185, length=0.012),
        mass=0.02,
    )

    right_focus_ring = model.part("right_focus_ring")
    right_focus_ring.visual(
        focus_ring_shell,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="right_focus_shell",
    )
    right_focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0185, length=0.012),
        mass=0.02,
    )

    model.articulation(
        "center_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=left_half,
        child=right_half,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.4,
            lower=0.0,
            upper=0.5,
        ),
    )
    model.articulation(
        "left_focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=left_half,
        child=left_focus_ring,
        origin=Origin(xyz=(-BARREL_CENTER_X, FOCUS_RING_CENTER_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )
    model.articulation(
        "right_focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=right_half,
        child=right_focus_ring,
        origin=Origin(xyz=(BARREL_CENTER_X, FOCUS_RING_CENTER_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_half = object_model.get_part("left_half")
    right_half = object_model.get_part("right_half")
    left_focus_ring = object_model.get_part("left_focus_ring")
    right_focus_ring = object_model.get_part("right_focus_ring")
    fold_hinge = object_model.get_articulation("center_fold_hinge")
    left_focus_joint = object_model.get_articulation("left_focus_rotation")
    right_focus_joint = object_model.get_articulation("right_focus_rotation")

    ctx.check(
        "central hinge is a vertical revolute joint",
        fold_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(fold_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={fold_hinge.articulation_type}, axis={fold_hinge.axis}",
    )
    ctx.check(
        "focus rings rotate continuously about the barrel axes",
        left_focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_focus_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(right_focus_joint.axis) == (0.0, 1.0, 0.0),
        details=(
            f"left={left_focus_joint.articulation_type}/{left_focus_joint.axis}, "
            f"right={right_focus_joint.articulation_type}/{right_focus_joint.axis}"
        ),
    )

    with ctx.pose({fold_hinge: 0.0}):
        ctx.expect_gap(
            right_half,
            left_half,
            axis="x",
            positive_elem="right_barrel_shell",
            negative_elem="left_barrel_shell",
            min_gap=0.012,
            max_gap=0.022,
            name="open barrels stay narrowly spaced across the bridge",
        )
        ctx.expect_overlap(
            left_half,
            right_half,
            axes="yz",
            elem_a="left_barrel_shell",
            elem_b="right_barrel_shell",
            min_overlap=0.020,
            name="open barrels remain aligned for viewing",
        )
        ctx.expect_overlap(
            left_focus_ring,
            left_half,
            axes="xyz",
            elem_a="left_focus_shell",
            elem_b="left_barrel_shell",
            min_overlap=0.012,
            name="left focus ring is mounted on the left eyepiece",
        )
        ctx.expect_overlap(
            right_focus_ring,
            right_half,
            axes="xyz",
            elem_a="right_focus_shell",
            elem_b="right_barrel_shell",
            min_overlap=0.012,
            name="right focus ring is mounted on the right eyepiece",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    rest_center = _aabb_center(ctx.part_element_world_aabb(right_half, elem="right_barrel_shell"))
    with ctx.pose({fold_hinge: 0.42}):
        folded_center = _aabb_center(ctx.part_element_world_aabb(right_half, elem="right_barrel_shell"))

    ctx.check(
        "right barrel folds inward around the central pivot",
        rest_center is not None
        and folded_center is not None
        and folded_center[0] < rest_center[0] - 0.0015
        and folded_center[1] > rest_center[1] + 0.006,
        details=f"rest_center={rest_center}, folded_center={folded_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
