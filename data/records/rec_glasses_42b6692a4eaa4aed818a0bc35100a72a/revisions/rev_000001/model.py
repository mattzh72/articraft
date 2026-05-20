from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


METAL = Material("warm_brushed_metal", rgba=(0.86, 0.66, 0.34, 1.0))
LENS = Material("pale_blue_transparent_lens", rgba=(0.58, 0.78, 0.92, 0.35))
NOSE_PAD = Material("translucent_silicone_pad", rgba=(0.92, 0.95, 0.90, 0.55))
TEMPLE_TIP = Material("dark_rubber_temple_tip", rgba=(0.02, 0.018, 0.015, 1.0))


RIGHT_LENS_CONTROLS = [
    (-0.022, 0.021),
    (-0.003, 0.027),
    (0.021, 0.020),
    (0.030, -0.004),
    (0.018, -0.027),
    (0.000, -0.034),
    (-0.020, -0.023),
    (-0.028, 0.002),
]


def _lens_profile(sign: int) -> list[tuple[float, float]]:
    """Smooth aviator lens outline in local X/Z coordinates."""
    controls = RIGHT_LENS_CONTROLS
    if sign < 0:
        controls = [(-x, z) for x, z in reversed(RIGHT_LENS_CONTROLS)]
    return sample_catmull_rom_spline_2d(
        controls,
        samples_per_segment=7,
        closed=True,
        alpha=0.5,
    )


def _thin_profile_mesh(profile_xz: list[tuple[float, float]], thickness_y: float) -> MeshGeometry:
    """Extrude an X/Z outline into a thin transparent lens plate."""
    geom = MeshGeometry()
    half = thickness_y / 2.0
    front = [geom.add_vertex(x, -half, z) for x, z in profile_xz]
    back = [geom.add_vertex(x, half, z) for x, z in profile_xz]
    n = len(profile_xz)

    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])

    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], front[j], back[j])
        geom.add_face(front[i], back[j], back[i])

    return geom


def _front_wire_mesh() -> MeshGeometry:
    """Continuous metal wire rims, double bridge, and bridge-to-rim joints."""
    geom = MeshGeometry()
    lens_centers = (-0.036, 0.036)
    for cx, sign in zip(lens_centers, (-1, 1)):
        rim = [(cx + x, 0.0, z) for x, z in _lens_profile(sign)]
        geom.merge(
            tube_from_spline_points(
                rim,
                radius=0.00165,
                samples_per_segment=3,
                closed_spline=True,
                radial_segments=16,
                up_hint=(0.0, 1.0, 0.0),
            )
        )

    upper_bridge = [
        (-0.016, 0.0, 0.021),
        (-0.008, 0.0, 0.026),
        (0.0, 0.0, 0.027),
        (0.008, 0.0, 0.026),
        (0.016, 0.0, 0.021),
    ]
    lower_bridge = [
        (-0.009, 0.0, 0.004),
        (-0.004, 0.0, 0.007),
        (0.0, 0.0, 0.008),
        (0.004, 0.0, 0.007),
        (0.009, 0.0, 0.004),
    ]
    for name_bridge in (upper_bridge, lower_bridge):
        geom.merge(
            tube_from_spline_points(
                name_bridge,
                radius=0.00145,
                samples_per_segment=8,
                radial_segments=14,
                cap_ends=True,
                up_hint=(0.0, 1.0, 0.0),
            )
        )

    return geom


def _temple_wire_mesh() -> MeshGeometry:
    """Slim metal temple arm whose local origin lies on the hinge axis."""
    return tube_from_spline_points(
        [
            (0.0, 0.0038, 0.000),
            (0.0, 0.038, 0.001),
            (0.0, 0.084, -0.002),
            (0.0, 0.118, -0.011),
        ],
        radius=0.00125,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )


def _temple_tip_mesh() -> MeshGeometry:
    """Slightly thicker rubber sleeve at the ear end."""
    return tube_from_spline_points(
        [
            (0.0, 0.086, -0.002),
            (0.0, 0.104, -0.006),
            (0.0, 0.124, -0.015),
        ],
        radius=0.0021,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )


def _nose_arm_mesh(sign: int) -> MeshGeometry:
    """Short adjustable metal arm from bridge pivot to silicone pad."""
    return tube_from_spline_points(
        [
            (0.0, 0.0018, 0.000),
            (-sign * 0.0025, 0.0055, -0.004),
            (-sign * 0.0065, 0.0090, -0.009),
        ],
        radius=0.00075,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aviator_glasses_double_bridge")

    front = model.part("front_frame")
    front.visual(
        mesh_from_geometry(_front_wire_mesh(), "front_rims"),
        material=METAL,
        name="front_rims",
    )

    # Compact hinge blocks at the two outer lens corners.  They are slightly
    # proud of the back of the rim so the temple leaves can touch without
    # being embedded.
    for idx, x in enumerate((-0.066, 0.066)):
        front.visual(
            Box((0.006, 0.006, 0.012)),
            origin=Origin(xyz=(x, 0.003, 0.004)),
            material=METAL,
            name=f"hinge_block_{idx}",
        )

    # Small pivot barrels on the bridge carry the adjustable nose-pad arms.
    for idx, x in enumerate((-0.0125, 0.0125)):
        front.visual(
            Cylinder(radius=0.0014, length=0.0055),
            origin=Origin(xyz=(x, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=METAL,
            name=f"pad_pivot_{idx}",
        )

    for idx, (cx, sign) in enumerate(((-0.036, -1), (0.036, 1))):
        lens = model.part(f"lens_{idx}")
        lens.visual(
            mesh_from_geometry(_thin_profile_mesh(_lens_profile(sign), 0.0016), f"lens_{idx}_shell"),
            material=LENS,
            name="lens_shell",
        )
        model.articulation(
            f"front_to_lens_{idx}",
            ArticulationType.FIXED,
            parent=front,
            child=lens,
            origin=Origin(xyz=(cx, 0.0, 0.0)),
        )

    for idx, x in enumerate((-0.066, 0.066)):
        temple = model.part(f"temple_{idx}")
        temple.visual(
            Box((0.0048, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=METAL,
            name="hinge_leaf",
        )
        temple.visual(
            mesh_from_geometry(_temple_wire_mesh(), f"temple_{idx}_wire"),
            material=METAL,
            name="temple_wire",
        )
        temple.visual(
            mesh_from_geometry(_temple_tip_mesh(), f"temple_{idx}_tip"),
            material=TEMPLE_TIP,
            name="ear_tip",
        )

        axis = (0.0, 0.0, -1.0) if idx == 0 else (0.0, 0.0, 1.0)
        model.articulation(
            f"front_to_temple_{idx}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=temple,
            origin=Origin(xyz=(x, 0.006, 0.004)),
            axis=axis,
            motion_limits=MotionLimits(effort=0.5, velocity=2.0, lower=0.0, upper=1.55),
        )

    for idx, (x, sign) in enumerate(((-0.0125, -1), (0.0125, 1))):
        pad = model.part(f"nose_pad_{idx}")
        pad.visual(
            Cylinder(radius=0.00155, length=0.004),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=METAL,
            name="pivot_eye",
        )
        pad.visual(
            mesh_from_geometry(_nose_arm_mesh(sign), f"nose_pad_{idx}_arm"),
            material=METAL,
            name="support_arm",
        )
        pad.visual(
            Box((0.0032, 0.0032, 0.0032)),
            origin=Origin(xyz=(-sign * 0.0068, 0.0096, -0.0100)),
            material=METAL,
            name="pad_boss",
        )
        pad.visual(
            mesh_from_geometry(
                _thin_profile_mesh(rounded_rect_profile(0.007, 0.014, 0.0025, corner_segments=5), 0.002),
                f"nose_pad_{idx}_cushion",
            ),
            origin=Origin(
                xyz=(-sign * 0.0070, 0.0100, -0.0110),
                rpy=(math.pi / 2.0, 0.0, sign * 0.26),
            ),
            material=NOSE_PAD,
            name="pad_cushion",
        )
        model.articulation(
            f"front_to_nose_pad_{idx}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=pad,
            origin=Origin(xyz=(x, 0.0, -0.004)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.08, velocity=1.2, lower=-0.35, upper=0.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # The glass lenses are intentionally seated through the circular wire rim
    # centerlines, and the nose-pad pivot eyes share the small parent pivot
    # barrels just like a captured screw/pin joint.
    for idx in (0, 1):
        ctx.allow_overlap(
            "front_frame",
            f"lens_{idx}",
            elem_a="front_rims",
            elem_b="lens_shell",
            reason="The transparent lens edge is intentionally captured in the thin metal rim groove.",
        )
        ctx.expect_overlap(
            f"lens_{idx}",
            "front_frame",
            axes="xz",
            elem_a="lens_shell",
            elem_b="front_rims",
            min_overlap=0.045,
            name=f"lens_{idx} is held inside its rim outline",
        )

        ctx.allow_overlap(
            "front_frame",
            f"nose_pad_{idx}",
            elem_a=f"pad_pivot_{idx}",
            elem_b="pivot_eye",
            reason="The nose-pad pivot eye is intentionally captured on the tiny bridge-mounted pivot barrel.",
        )
        ctx.expect_overlap(
            f"nose_pad_{idx}",
            "front_frame",
            axes="xyz",
            elem_a="pivot_eye",
            elem_b=f"pad_pivot_{idx}",
            min_overlap=0.001,
            name=f"nose_pad_{idx} pivot eye remains captured",
        )

    ctx.expect_contact(
        "temple_0",
        "front_frame",
        elem_a="hinge_leaf",
        elem_b="hinge_block_0",
        contact_tol=0.0007,
        name="temple_0 hinge leaf seats on compact hinge block",
    )
    ctx.expect_contact(
        "temple_1",
        "front_frame",
        elem_a="hinge_leaf",
        elem_b="hinge_block_1",
        contact_tol=0.0007,
        name="temple_1 hinge leaf seats on compact hinge block",
    )

    temple_0 = object_model.get_part("temple_0")
    temple_1 = object_model.get_part("temple_1")
    joint_0 = object_model.get_articulation("front_to_temple_0")
    joint_1 = object_model.get_articulation("front_to_temple_1")
    rest_tip_0 = ctx.part_element_world_aabb(temple_0, elem="ear_tip")
    rest_tip_1 = ctx.part_element_world_aabb(temple_1, elem="ear_tip")
    with ctx.pose({joint_0: 1.55, joint_1: 1.55}):
        folded_tip_0 = ctx.part_element_world_aabb(temple_0, elem="ear_tip")
        folded_tip_1 = ctx.part_element_world_aabb(temple_1, elem="ear_tip")

    ctx.check(
        "temples fold inward across the lenses",
        rest_tip_0 is not None
        and rest_tip_1 is not None
        and folded_tip_0 is not None
        and folded_tip_1 is not None
        and folded_tip_0[1][0] > 0.025
        and folded_tip_1[0][0] < -0.025,
        details=f"rest0={rest_tip_0}, folded0={folded_tip_0}, rest1={rest_tip_1}, folded1={folded_tip_1}",
    )

    nose_joint_0 = object_model.get_articulation("front_to_nose_pad_0")
    pad_0 = object_model.get_part("nose_pad_0")
    rest_pad = ctx.part_element_world_aabb(pad_0, elem="pad_cushion")
    with ctx.pose({nose_joint_0: 0.35}):
        rocked_pad = ctx.part_element_world_aabb(pad_0, elem="pad_cushion")
    ctx.check(
        "nose pad rotates on support-arm pivot",
        rest_pad is not None and rocked_pad is not None and rocked_pad[1][1] > rest_pad[1][1] + 0.001,
        details=f"rest={rest_pad}, rocked={rocked_pad}",
    )

    return ctx.report()


object_model = build_object_model()
