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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anamorphic_taking_lens")

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    engrave_white = model.material("engrave_white", rgba=(0.92, 0.92, 0.90, 1.0))

    body_shell = _mesh(
        "fixed_body_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.026, -0.102),
                (0.030, -0.096),
                (0.030, -0.088),
                (0.038, -0.088),
                (0.038, -0.036),
                (0.040, -0.032),
                (0.040, 0.034),
                (0.0402, 0.040),
                (0.0402, 0.078),
                (0.0355, 0.086),
            ],
            [
                (0.021, -0.100),
                (0.021, -0.090),
                (0.023, -0.088),
                (0.023, -0.038),
                (0.0255, -0.032),
                (0.0255, 0.034),
                (0.023, 0.040),
                (0.023, 0.080),
                (0.0195, 0.086),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    front_shell = _mesh(
        "front_anamorphic_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.046, -0.024),
                (0.0475, -0.020),
                (0.0475, -0.006),
                (0.049, 0.000),
                (0.049, 0.020),
                (0.0505, 0.024),
                (0.0505, 0.050),
                (0.052, 0.056),
                (0.052, 0.068),
            ],
            [
                (0.0408, -0.022),
                (0.0408, 0.020),
                (0.0365, 0.026),
                (0.0365, 0.060),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    rear_shell = _mesh(
        "rear_focus_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0415, -0.028),
                (0.0435, -0.024),
                (0.0435, -0.012),
                (0.045, -0.010),
                (0.045, 0.010),
                (0.0435, 0.012),
                (0.0435, 0.022),
                (0.0415, 0.030),
            ],
            [
                (0.0391, -0.026),
                (0.0391, -0.010),
                (0.0391, 0.010),
                (0.0391, 0.026),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    front_retainer_ring = _mesh(
        "front_retainer_ring",
        LatheGeometry.from_shell_profiles(
            [
                (0.046, -0.002),
                (0.046, 0.002),
            ],
            [
                (0.0398, -0.002),
                (0.0398, 0.002),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    rear_retainer_ring = _mesh(
        "rear_retainer_ring",
        LatheGeometry.from_shell_profiles(
            [
                (0.0415, -0.002),
                (0.0415, 0.002),
            ],
            [
                (0.0375, -0.002),
                (0.0375, 0.002),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    fixed_body = model.part("fixed_body")
    fixed_body.visual(body_shell, material=anodized_black, name="body_shell")
    fixed_body.visual(
        front_retainer_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=anodized_black,
        name="front_retainer_ring",
    )
    fixed_body.visual(
        rear_retainer_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=anodized_black,
        name="rear_retainer_ring",
    )
    fixed_body.visual(
        Box((0.0025, 0.008, 0.024)),
        origin=Origin(xyz=(0.0412, 0.0, 0.004)),
        material=engrave_white,
        name="body_index_mark",
    )
    fixed_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.188),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
    )

    front_anamorphic = model.part("front_anamorphic")
    front_anamorphic.visual(front_shell, material=satin_black, name="front_shell")
    front_anamorphic.visual(
        Box((0.004, 0.010, 0.018)),
        origin=Origin(xyz=(0.0517, 0.0, 0.022)),
        material=engrave_white,
        name="front_alignment_mark",
    )
    front_anamorphic.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.092),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    rear_focus_barrel = model.part("rear_focus_barrel")
    rear_focus_barrel.visual(rear_shell, material=rubber_black, name="rear_shell")
    rear_focus_barrel.visual(
        Box((0.004, 0.011, 0.020)),
        origin=Origin(xyz=(0.0445, 0.0, -0.002)),
        material=engrave_white,
        name="focus_distance_mark",
    )
    rear_focus_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.060),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
    )

    model.articulation(
        "body_to_front_anamorphic",
        ArticulationType.REVOLUTE,
        parent=fixed_body,
        child=front_anamorphic,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=1.5,
            lower=-math.radians(35.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "body_to_rear_focus",
        ArticulationType.REVOLUTE,
        parent=fixed_body,
        child=rear_focus_barrel,
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(300.0),
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

    body = object_model.get_part("fixed_body")
    front = object_model.get_part("front_anamorphic")
    rear = object_model.get_part("rear_focus_barrel")
    front_joint = object_model.get_articulation("body_to_front_anamorphic")
    rear_joint = object_model.get_articulation("body_to_rear_focus")

    body_shell = body.get_visual("body_shell")
    front_shell = front.get_visual("front_shell")
    rear_shell = rear.get_visual("rear_shell")

    ctx.check(
        "front element rotates about optical axis",
        front_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={front_joint.axis}",
    )
    ctx.check(
        "rear focus barrel rotates about optical axis",
        rear_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={rear_joint.axis}",
    )

    front_limits = front_joint.motion_limits
    rear_limits = rear_joint.motion_limits
    ctx.check(
        "front element can align around neutral",
        front_limits is not None
        and front_limits.lower is not None
        and front_limits.upper is not None
        and front_limits.lower < 0.0 < front_limits.upper,
        details=f"limits={front_limits}",
    )
    ctx.check(
        "rear focus barrel has substantial throw",
        rear_limits is not None
        and rear_limits.lower is not None
        and rear_limits.upper is not None
        and (rear_limits.upper - rear_limits.lower) >= math.radians(240.0),
        details=f"limits={rear_limits}",
    )

    ctx.expect_origin_distance(
        front,
        body,
        axes="xy",
        max_dist=1e-6,
        name="front element stays coaxial with fixed body",
    )
    ctx.expect_origin_distance(
        rear,
        body,
        axes="xy",
        max_dist=1e-6,
        name="rear focus barrel stays coaxial with fixed body",
    )
    ctx.expect_origin_gap(
        front,
        rear,
        axis="z",
        min_gap=0.10,
        name="front anamorphic section sits ahead of focus barrel",
    )
    ctx.expect_overlap(
        front,
        body,
        axes="xy",
        elem_a=front_shell,
        elem_b=body_shell,
        min_overlap=0.075,
        name="front sleeve remains centered over the body",
    )
    ctx.expect_overlap(
        rear,
        body,
        axes="xy",
        elem_a=rear_shell,
        elem_b=body_shell,
        min_overlap=0.070,
        name="rear focus sleeve remains centered over the body",
    )

    with ctx.pose(
        {
            front_joint: math.radians(28.0),
            rear_joint: math.radians(220.0),
        }
    ):
        ctx.expect_origin_distance(
            front,
            body,
            axes="xy",
            max_dist=1e-6,
            name="front element remains coaxial while rotated",
        )
        ctx.expect_origin_distance(
            rear,
            body,
            axes="xy",
            max_dist=1e-6,
            name="rear focus barrel remains coaxial while rotated",
        )
        ctx.expect_overlap(
            front,
            body,
            axes="xy",
            elem_a=front_shell,
            elem_b=body_shell,
            min_overlap=0.075,
            name="front sleeve keeps radial registration while rotated",
        )
        ctx.expect_overlap(
            rear,
            body,
            axes="xy",
            elem_a=rear_shell,
            elem_b=body_shell,
            min_overlap=0.070,
            name="rear focus sleeve keeps radial registration while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
