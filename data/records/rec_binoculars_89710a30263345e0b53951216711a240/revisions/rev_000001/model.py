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


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_pt, max_pt) = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))


def _build_barrel_mesh():
    profile = [
        (0.0, -0.034),
        (0.018, -0.034),
        (0.020, -0.028),
        (0.0215, -0.016),
        (0.0245, 0.006),
        (0.0265, 0.050),
        (0.0290, 0.100),
        (0.0325, 0.145),
        (0.0345, 0.162),
        (0.0300, 0.178),
        (0.0, 0.178),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=52), "binocular_barrel")


def _build_diopter_ring_mesh():
    outer_profile = [
        (0.0243, -0.0070),
        (0.0253, -0.0045),
        (0.0253, 0.0045),
        (0.0243, 0.0070),
    ]
    inner_profile = [
        (0.0215, -0.0070),
        (0.0215, 0.0070),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "diopter_ring_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laser_rangefinding_binocular")

    armor = model.material("armor", rgba=(0.16, 0.18, 0.16, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.09, 0.10, 1.0))
    metal = model.material("metal", rgba=(0.24, 0.25, 0.27, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.19, 0.20, 0.22, 1.0))
    button_orange = model.material("button_orange", rgba=(0.83, 0.43, 0.16, 1.0))

    barrel_mesh = _build_barrel_mesh()
    diopter_mesh = _build_diopter_ring_mesh()

    body = model.part("body")
    body.visual(
        barrel_mesh,
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor,
        name="left_objective_housing",
    )
    body.visual(
        barrel_mesh,
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor,
        name="right_objective_housing",
    )
    body.visual(
        Cylinder(radius=0.0355, length=0.010),
        origin=Origin(xyz=(0.167, 0.040, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="left_objective_bezel",
    )
    body.visual(
        Cylinder(radius=0.0355, length=0.010),
        origin=Origin(xyz=(0.167, -0.040, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="right_objective_bezel",
    )
    body.visual(
        Box((0.086, 0.070, 0.028)),
        origin=Origin(xyz=(0.060, 0.0, 0.004)),
        material=armor,
        name="center_bridge",
    )
    body.visual(
        Box((0.050, 0.058, 0.024)),
        origin=Origin(xyz=(-0.002, 0.0, 0.004)),
        material=armor,
        name="rear_bridge",
    )
    body.visual(
        Cylinder(radius=0.0205, length=0.018),
        origin=Origin(xyz=(-0.041, 0.040, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="left_eyepiece_core",
    )
    body.visual(
        Cylinder(radius=0.0205, length=0.018),
        origin=Origin(xyz=(-0.041, -0.040, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="right_eyepiece_core",
    )
    body.visual(
        Cylinder(radius=0.0230, length=0.006),
        origin=Origin(xyz=(-0.031, 0.040, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="left_eyepiece_shoulder",
    )
    body.visual(
        Cylinder(radius=0.0230, length=0.006),
        origin=Origin(xyz=(-0.031, -0.040, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="right_eyepiece_shoulder",
    )
    body.visual(
        Box((0.052, 0.028, 0.020)),
        origin=Origin(xyz=(0.104, 0.0, -0.013)),
        material=armor,
        name="lower_saddle",
    )
    body.visual(
        Box((0.060, 0.036, 0.022)),
        origin=Origin(xyz=(0.060, 0.0, 0.035)),
        material=metal,
        name="display_prism_base",
    )
    body.visual(
        Box((0.036, 0.026, 0.014)),
        origin=Origin(xyz=(0.060, 0.0, 0.023)),
        material=metal,
        name="display_prism_neck",
    )
    body.visual(
        Box((0.036, 0.022, 0.012)),
        origin=Origin(xyz=(0.060, 0.0, 0.051)),
        material=gunmetal,
        name="display_prism_cap",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.063, 0.0, 0.059)),
        material=gunmetal,
        name="range_button_pedestal",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(xyz=(0.063, 0.0, 0.063)),
        material=button_orange,
        name="range_button",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.174, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="laser_aperture",
    )
    body.visual(
        Box((0.007, 0.010, 0.010)),
        origin=Origin(xyz=(0.041, 0.016, 0.055)),
        material=metal,
        name="hinge_ear_left",
    )
    body.visual(
        Box((0.007, 0.010, 0.010)),
        origin=Origin(xyz=(0.041, -0.016, 0.055)),
        material=metal,
        name="hinge_ear_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.205, 0.150, 0.082)),
        mass=1.15,
        origin=Origin(xyz=(0.065, 0.0, 0.010)),
    )

    diopter_ring = model.part("diopter_ring")
    diopter_ring.visual(
        diopter_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="diopter_shell",
    )
    diopter_ring.visual(
        Box((0.0035, 0.0065, 0.0024)),
        origin=Origin(xyz=(0.0, 0.0, 0.0250)),
        material=metal,
        name="diopter_index_mark",
    )
    diopter_ring.inertial = Inertial.from_geometry(
        Box((0.016, 0.052, 0.052)),
        mass=0.03,
        origin=Origin(),
    )

    button_guard = model.part("button_guard")
    button_guard.visual(
        Cylinder(radius=0.003, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="guard_hinge_barrel",
    )
    button_guard.visual(
        Box((0.030, 0.003, 0.008)),
        origin=Origin(xyz=(0.018, 0.0105, 0.0015)),
        material=metal,
        name="guard_side_left",
    )
    button_guard.visual(
        Box((0.030, 0.003, 0.008)),
        origin=Origin(xyz=(0.018, -0.0105, 0.0015)),
        material=metal,
        name="guard_side_right",
    )
    button_guard.visual(
        Box((0.040, 0.024, 0.003)),
        origin=Origin(xyz=(0.024, 0.0, 0.007)),
        material=metal,
        name="guard_cap",
    )
    button_guard.visual(
        Box((0.004, 0.024, 0.009)),
        origin=Origin(xyz=(0.046, 0.0, 0.0020)),
        material=metal,
        name="guard_front_lip",
    )
    button_guard.inertial = Inertial.from_geometry(
        Box((0.036, 0.026, 0.016)),
        mass=0.025,
        origin=Origin(xyz=(0.016, 0.0, 0.004)),
    )

    model.articulation(
        "body_to_diopter_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=diopter_ring,
        origin=Origin(xyz=(-0.041, -0.040, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    model.articulation(
        "body_to_button_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=button_guard,
        origin=Origin(xyz=(0.041, 0.0, 0.061)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    diopter_ring = object_model.get_part("diopter_ring")
    button_guard = object_model.get_part("button_guard")
    diopter_joint = object_model.get_articulation("body_to_diopter_ring")
    guard_joint = object_model.get_articulation("body_to_button_guard")

    ctx.check(
        "diopter ring uses continuous rotation",
        diopter_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(diopter_joint.axis) == (1.0, 0.0, 0.0)
        and diopter_joint.motion_limits is not None
        and diopter_joint.motion_limits.lower is None
        and diopter_joint.motion_limits.upper is None,
        details=(
            f"type={diopter_joint.articulation_type}, axis={diopter_joint.axis}, "
            f"limits={diopter_joint.motion_limits}"
        ),
    )
    ctx.check(
        "button guard hinge opens upward",
        guard_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(guard_joint.axis) == (0.0, -1.0, 0.0)
        and guard_joint.motion_limits is not None
        and guard_joint.motion_limits.lower == 0.0
        and guard_joint.motion_limits.upper is not None
        and guard_joint.motion_limits.upper >= 1.2,
        details=(
            f"type={guard_joint.articulation_type}, axis={guard_joint.axis}, "
            f"limits={guard_joint.motion_limits}"
        ),
    )

    with ctx.pose({guard_joint: 0.0}):
        ctx.expect_gap(
            button_guard,
            body,
            axis="z",
            positive_elem="guard_cap",
            negative_elem="range_button",
            min_gap=0.0,
            max_gap=0.004,
            name="closed guard hovers just above range button",
        )
        ctx.expect_overlap(
            button_guard,
            body,
            axes="xy",
            elem_a="guard_cap",
            elem_b="range_button",
            min_overlap=0.010,
            name="closed guard covers the range button",
        )
        ctx.expect_overlap(
            diopter_ring,
            body,
            axes="yz",
            elem_a="diopter_shell",
            elem_b="right_objective_housing",
            min_overlap=0.030,
            name="diopter ring stays concentric with the right ocular housing",
        )
        ctx.expect_contact(
            diopter_ring,
            body,
            elem_a="diopter_shell",
            elem_b="right_eyepiece_shoulder",
            contact_tol=1e-6,
            name="diopter ring seats against the right eyepiece shoulder",
        )

        closed_guard_cap = _aabb_center(ctx.part_element_world_aabb(button_guard, elem="guard_cap"))
        closed_ring_mark = _aabb_center(
            ctx.part_element_world_aabb(diopter_ring, elem="diopter_index_mark")
        )

    with ctx.pose({guard_joint: 1.1}):
        opened_guard_cap = _aabb_center(ctx.part_element_world_aabb(button_guard, elem="guard_cap"))

    with ctx.pose({diopter_joint: 1.1}):
        rotated_ring_mark = _aabb_center(
            ctx.part_element_world_aabb(diopter_ring, elem="diopter_index_mark")
        )

    ctx.check(
        "button guard cap lifts clear when opened",
        closed_guard_cap is not None
        and opened_guard_cap is not None
        and opened_guard_cap[2] > closed_guard_cap[2] + 0.012,
        details=f"closed={closed_guard_cap}, opened={opened_guard_cap}",
    )
    ctx.check(
        "diopter index mark moves around optical axis",
        closed_ring_mark is not None
        and rotated_ring_mark is not None
        and (
            abs(rotated_ring_mark[1] - closed_ring_mark[1]) > 0.008
            or abs(rotated_ring_mark[2] - closed_ring_mark[2]) > 0.008
        ),
        details=f"closed={closed_ring_mark}, rotated={rotated_ring_mark}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
