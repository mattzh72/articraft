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
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_rounded_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    corner = min(radius, width * 0.48, height * 0.48)
    return [
        (x_pos, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, corner, corner_segments=8)
    ]


def _x_shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 64,
    start_cap: str = "flat",
    end_cap: str = "flat",
    lip_samples: int = 8,
):
    return _mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap=start_cap,
            end_cap=end_cap,
            lip_samples=lip_samples,
        ).rotate_y(math.pi / 2.0),
    )


def _element_center(ctx: TestContext, part, elem: str):
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monocular_spotter")

    armor_graphite = model.material("armor_graphite", rgba=(0.17, 0.18, 0.19, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.34, 0.36, 0.39, 1.0))

    body = model.part("body")

    prism_body = section_loft(
        [
            _yz_rounded_section(-0.008, width=0.054, height=0.060, radius=0.010, z_center=0.000),
            _yz_rounded_section(0.016, width=0.066, height=0.076, radius=0.014, z_center=0.002),
            _yz_rounded_section(0.052, width=0.080, height=0.094, radius=0.018, z_center=0.005),
            _yz_rounded_section(0.086, width=0.076, height=0.088, radius=0.016, z_center=0.004),
            _yz_rounded_section(0.118, width=0.062, height=0.074, radius=0.014, z_center=0.001),
        ]
    )
    body.visual(
        _mesh("prism_body_shell", prism_body),
        material=armor_graphite,
        name="prism_body_shell",
    )

    body.visual(
        _x_shell_mesh(
            "objective_housing_shell",
            [
                (0.031, 0.090),
                (0.033, 0.100),
                (0.036, 0.112),
                (0.037, 0.208),
                (0.039, 0.220),
                (0.039, 0.228),
            ],
            [
                (0.026, 0.092),
                (0.028, 0.104),
                (0.031, 0.114),
                (0.032, 0.214),
                (0.034, 0.224),
            ],
            segments=72,
        ),
        material=satin_black,
        name="objective_housing_shell",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.135),
        origin=Origin(xyz=(0.1575, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="objective_baffle",
    )
    body.visual(
        _mesh(
            "rear_collar",
            LatheGeometry(
                [
                    (0.0, -0.022),
                    (0.024, -0.022),
                    (0.024, 0.022),
                    (0.0, 0.022),
                ],
                segments=72,
            ).rotate_y(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=trim_gray,
        name="rear_collar",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.040, 0.0, -0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="underbody_bridge",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.250, 0.090, 0.100)),
        mass=0.68,
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _x_shell_mesh(
            "focus_ring_shell",
            [
                (0.0245, -0.074),
                (0.0245, -0.052),
                (0.0280, -0.048),
                (0.0315, -0.044),
                (0.0315, -0.010),
                (0.0290, -0.004),
                (0.0275, 0.000),
            ],
            [
                (0.0190, -0.074),
                (0.0190, -0.051),
                (0.0215, -0.048),
                (0.0240, -0.044),
                (0.0240, 0.000),
            ],
            segments=72,
        ),
        material=rubber_black,
        name="focus_ring_shell",
    )
    focus_ring.visual(
        Box((0.010, 0.004, 0.003)),
        origin=Origin(xyz=(-0.025, 0.0, 0.031)),
        material=trim_gray,
        name="focus_index",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Box((0.078, 0.066, 0.066)),
        mass=0.08,
        origin=Origin(xyz=(-0.037, 0.0, 0.0)),
    )

    eyecup = model.part("eyecup")
    eyecup.visual(
        _x_shell_mesh(
            "eyecup_shell",
            [
                (0.0275, -0.036),
                (0.0280, -0.020),
                (0.0310, -0.010),
                (0.0355, -0.003),
                (0.0365, 0.000),
            ],
            [
                (0.0185, -0.034),
                (0.0205, -0.018),
                (0.0234, -0.008),
                (0.0246, 0.000),
            ],
            segments=72,
        ),
        material=rubber_black,
        name="eyecup_shell",
    )
    eyecup.visual(
        Box((0.008, 0.003, 0.003)),
        origin=Origin(xyz=(-0.010, 0.0, 0.034)),
        material=trim_gray,
        name="eyecup_index",
    )
    eyecup.inertial = Inertial.from_geometry(
        Box((0.040, 0.075, 0.075)),
        mass=0.03,
        origin=Origin(xyz=(-0.018, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=10.0),
    )
    model.articulation(
        "focus_ring_to_eyecup",
        ArticulationType.REVOLUTE,
        parent=focus_ring,
        child=eyecup,
        origin=Origin(xyz=(-0.072, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=0.0,
            upper=4.3,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    focus_ring = object_model.get_part("focus_ring")
    eyecup = object_model.get_part("eyecup")
    focus_joint = object_model.get_articulation("body_to_focus_ring")
    eyecup_joint = object_model.get_articulation("focus_ring_to_eyecup")

    ctx.check(
        "focus ring uses continuous optical-axis rotation",
        focus_joint.articulation_type == ArticulationType.CONTINUOUS and focus_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={focus_joint.articulation_type}, axis={focus_joint.axis}",
    )
    ctx.check(
        "eyecup uses revolute optical-axis twist",
        eyecup_joint.articulation_type == ArticulationType.REVOLUTE
        and eyecup_joint.axis == (1.0, 0.0, 0.0)
        and eyecup_joint.motion_limits is not None
        and eyecup_joint.motion_limits.lower == 0.0
        and eyecup_joint.motion_limits.upper is not None
        and eyecup_joint.motion_limits.upper >= 4.0,
        details=(
            f"type={eyecup_joint.articulation_type}, axis={eyecup_joint.axis}, "
            f"limits={eyecup_joint.motion_limits}"
        ),
    )

    ctx.expect_overlap(
        focus_ring,
        body,
        axes="yz",
        elem_a="focus_ring_shell",
        elem_b="rear_collar",
        min_overlap=0.045,
        name="focus ring stays concentric with the rear collar",
    )
    ctx.expect_overlap(
        eyecup,
        focus_ring,
        axes="yz",
        elem_a="eyecup_shell",
        elem_b="focus_ring_shell",
        min_overlap=0.045,
        name="eyecup stays concentric with the eyepiece stack",
    )

    rest_focus_index = _element_center(ctx, focus_ring, "focus_index")
    with ctx.pose({focus_joint: math.pi / 2.0}):
        quarter_focus_index = _element_center(ctx, focus_ring, "focus_index")
    ctx.check(
        "focus ring index orbits around the optical axis",
        rest_focus_index is not None
        and quarter_focus_index is not None
        and rest_focus_index[2] > 0.025
        and quarter_focus_index[1] < -0.020
        and abs(quarter_focus_index[2]) < 0.010,
        details=f"rest={rest_focus_index}, quarter_turn={quarter_focus_index}",
    )

    rest_eyecup_index = _element_center(ctx, eyecup, "eyecup_index")
    with ctx.pose({eyecup_joint: math.pi}):
        half_turn_eyecup_index = _element_center(ctx, eyecup, "eyecup_index")
    ctx.check(
        "eyecup twist changes orientation about the eyepiece axis",
        rest_eyecup_index is not None
        and half_turn_eyecup_index is not None
        and rest_eyecup_index[2] > 0.026
        and half_turn_eyecup_index[2] < -0.026,
        details=f"rest={rest_eyecup_index}, half_turn={half_turn_eyecup_index}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
