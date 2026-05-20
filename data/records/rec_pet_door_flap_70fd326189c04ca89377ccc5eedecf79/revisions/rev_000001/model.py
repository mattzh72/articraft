from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rounded_rect_slab_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    radius: float,
    segments: int = 6,
) -> MeshGeometry:
    """Build a rounded rectangular slab in local X/Z, with thickness along Y."""
    half_w = width * 0.5
    half_h = height * 0.5
    r = min(radius, half_w * 0.45, half_h * 0.45)
    centers = (
        (half_w - r, half_h - r, 0.0, math.pi / 2.0),
        (-half_w + r, half_h - r, math.pi / 2.0, math.pi),
        (-half_w + r, -half_h + r, math.pi, 3.0 * math.pi / 2.0),
        (half_w - r, -half_h + r, 3.0 * math.pi / 2.0, 2.0 * math.pi),
    )
    outline: list[tuple[float, float]] = []
    for cx, cz, start, end in centers:
        for index in range(segments + 1):
            if outline and index == 0:
                continue
            t = index / segments
            angle = start + (end - start) * t
            outline.append((cx + r * math.cos(angle), cz + r * math.sin(angle)))

    geom = MeshGeometry()
    front_ids = [
        geom.add_vertex(x, -thickness * 0.5, z)
        for x, z in outline
    ]
    rear_ids = [
        geom.add_vertex(x, thickness * 0.5, z)
        for x, z in outline
    ]
    count = len(outline)
    for index in range(1, count - 1):
        geom.add_face(front_ids[0], front_ids[index], front_ids[index + 1])
        geom.add_face(rear_ids[0], rear_ids[index + 1], rear_ids[index])
    for index in range(count):
        nxt = (index + 1) % count
        geom.add_face(front_ids[index], rear_ids[index], rear_ids[nxt])
        geom.add_face(front_ids[index], rear_ids[nxt], front_ids[nxt])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dog_door_flap_tunnel")

    frame_plastic = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    frame_shadow = model.material("inner_shadow_grey", rgba=(0.36, 0.37, 0.36, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.64, 0.65, 0.62, 1.0))
    flap_tint = model.material("smoked_flexible_clear", rgba=(0.55, 0.37, 0.18, 0.48))
    panel_finish = model.material("security_panel_grey", rgba=(0.33, 0.38, 0.42, 1.0))
    rubber_black = model.material("black_rubber_seal", rgba=(0.05, 0.05, 0.045, 1.0))

    frame = model.part("tunnel_frame")

    # Four deep walls form the hollow pet-door tunnel. The wall boxes meet at
    # the corners, so the frame part reads as one continuous molded assembly.
    frame.visual(
        Box((0.080, 0.240, 0.820)),
        origin=Origin(xyz=(-0.300, 0.000, 0.410)),
        material=frame_plastic,
        name="side_wall_0",
    )
    frame.visual(
        Box((0.080, 0.240, 0.820)),
        origin=Origin(xyz=(0.300, 0.000, 0.410)),
        material=frame_plastic,
        name="side_wall_1",
    )
    frame.visual(
        Box((0.680, 0.240, 0.080)),
        origin=Origin(xyz=(0.000, 0.000, 0.780)),
        material=frame_plastic,
        name="top_wall",
    )
    frame.visual(
        Box((0.680, 0.240, 0.080)),
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
        material=frame_plastic,
        name="bottom_wall",
    )
    # Raised front trim ring makes the tunnel read as a through-wall pet door.
    frame.visual(
        Box((0.120, 0.035, 0.900)),
        origin=Origin(xyz=(-0.335, -0.135, 0.410)),
        material=frame_plastic,
        name="front_trim_0",
    )
    frame.visual(
        Box((0.120, 0.035, 0.900)),
        origin=Origin(xyz=(0.335, -0.135, 0.410)),
        material=frame_plastic,
        name="front_trim_1",
    )
    frame.visual(
        Box((0.790, 0.035, 0.120)),
        origin=Origin(xyz=(0.000, -0.135, 0.815)),
        material=frame_plastic,
        name="front_trim_top",
    )
    frame.visual(
        Box((0.790, 0.035, 0.120)),
        origin=Origin(xyz=(0.000, -0.135, 0.005)),
        material=frame_plastic,
        name="front_trim_bottom",
    )
    frame.visual(
        Box((0.500, 0.010, 0.018)),
        origin=Origin(xyz=(0.000, -0.156, 0.073)),
        material=rubber_black,
        name="lower_flap_seal",
    )

    # Rear side channels capture the sliding security panel. The lips overlap
    # the panel edges in projection but have clearance in Y, so the panel is
    # clipped without unintended interpenetration.
    for side, x in enumerate((-0.245, 0.245)):
        frame.visual(
            Box((0.030, 0.040, 1.220)),
            origin=Origin(xyz=(x, 0.070, 0.690)),
            material=frame_plastic,
            name=f"channel_web_{side}",
        )
        frame.visual(
            Box((0.056, 0.010, 1.220)),
            origin=Origin(xyz=(-0.232 if side == 0 else 0.232, 0.055, 0.690)),
            material=frame_plastic,
            name=f"front_lip_{side}",
        )
        frame.visual(
            Box((0.056, 0.010, 1.220)),
            origin=Origin(xyz=(-0.232 if side == 0 else 0.232, 0.085, 0.690)),
            material=frame_plastic,
            name=f"rear_lip_{side}",
        )
    frame.visual(
        Box((0.610, 0.050, 0.050)),
        origin=Origin(xyz=(0.000, 0.070, 1.305)),
        material=frame_plastic,
        name="channel_top_stop",
    )

    # Exposed hinge knuckles mounted to the upper trim edge.
    for side, x in enumerate((-0.205, 0.205)):
        frame.visual(
            Box((0.110, 0.010, 0.030)),
            origin=Origin(xyz=(x, -0.157, 0.744)),
            material=hinge_metal,
            name=f"hinge_leaf_{side}",
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.100),
            origin=Origin(xyz=(x, -0.171, 0.725), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"frame_hinge_barrel_{side}",
        )
    frame.visual(
        Cylinder(radius=0.006, length=0.520),
        origin=Origin(xyz=(0.000, -0.171, 0.725), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="hinge_pin",
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_geometry(
            _rounded_rect_slab_mesh(width=0.445, height=0.580, thickness=0.020, radius=0.035),
            "rounded_dog_flap",
        ),
        origin=Origin(xyz=(0.000, 0.000, -0.305)),
        material=flap_tint,
        name="flap_panel",
    )
    flap.visual(
        Box((0.455, 0.006, 0.016)),
        origin=Origin(xyz=(0.000, -0.013, -0.590)),
        material=rubber_black,
        name="bottom_gasket",
    )
    flap.visual(
        Box((0.300, 0.012, 0.025)),
        origin=Origin(xyz=(0.000, -0.006, -0.017)),
        material=hinge_metal,
        name="flap_hinge_leaf",
    )
    flap.visual(
        Cylinder(radius=0.014, length=0.260),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="flap_hinge_barrel",
    )

    security_panel = model.part("security_panel")
    security_panel.visual(
        mesh_from_geometry(
            _rounded_rect_slab_mesh(width=0.440, height=0.620, thickness=0.016, radius=0.018),
            "sliding_security_panel",
        ),
        origin=Origin(),
        material=panel_finish,
        name="panel_slab",
    )
    security_panel.visual(
        Box((0.180, 0.012, 0.045)),
        origin=Origin(xyz=(0.000, 0.014, -0.120)),
        material=hinge_metal,
        name="pull_grip",
    )
    security_panel.visual(
        Box((0.390, 0.004, 0.018)),
        origin=Origin(xyz=(0.000, -0.010, 0.260)),
        material=hinge_metal,
        name="stiffening_rib",
    )
    for side, x in enumerate((-0.224, 0.224)):
        security_panel.visual(
            Box((0.012, 0.020, 0.600)),
            origin=Origin(xyz=(x, 0.000, 0.000)),
            material=frame_shadow,
            name=f"side_runner_{side}",
        )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.000, -0.171, 0.725)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "frame_to_security_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=security_panel,
        origin=Origin(xyz=(0.000, 0.070, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.25, lower=0.0, upper=0.550),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("tunnel_frame")
    flap = object_model.get_part("flap")
    panel = object_model.get_part("security_panel")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    panel_slide = object_model.get_articulation("frame_to_security_panel")

    ctx.allow_overlap(
        frame,
        flap,
        elem_a="hinge_pin",
        elem_b="flap_hinge_barrel",
        reason="The hinge pin is intentionally captured inside the flap hinge barrel.",
    )
    ctx.expect_overlap(
        frame,
        flap,
        axes="x",
        elem_a="hinge_pin",
        elem_b="flap_hinge_barrel",
        min_overlap=0.24,
        name="flap hinge barrel is retained on the hinge pin",
    )

    flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "flap panel spans the pet opening",
        flap_aabb is not None
        and (flap_aabb[1][0] - flap_aabb[0][0]) > 0.42
        and (flap_aabb[1][2] - flap_aabb[0][2]) > 0.55,
        details=f"flap_panel_aabb={flap_aabb}",
    )
    ctx.expect_gap(
        frame,
        flap,
        axis="y",
        positive_elem="front_trim_top",
        negative_elem="flap_panel",
        min_gap=0.004,
        max_gap=0.020,
        name="flap is in front of the upper frame without collision",
    )

    def _check_panel_captured(prefix: str) -> None:
        ctx.expect_gap(
            panel,
            frame,
            axis="y",
            positive_elem="panel_slab",
            negative_elem="front_lip_0",
            min_gap=0.001,
            max_gap=0.008,
            name=f"{prefix} panel clears front channel lips",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="y",
            positive_elem="rear_lip_0",
            negative_elem="panel_slab",
            min_gap=0.001,
            max_gap=0.008,
            name=f"{prefix} panel clears rear channel lips",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xz",
            elem_a="panel_slab",
            elem_b="front_lip_0",
            min_overlap=0.012,
            name=f"{prefix} left edge is clipped behind the channel lip",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xz",
            elem_a="panel_slab",
            elem_b="front_lip_1",
            min_overlap=0.012,
            name=f"{prefix} right edge is clipped behind the channel lip",
        )
        ctx.expect_contact(
            panel,
            frame,
            elem_a="side_runner_0",
            elem_b="front_lip_0",
            contact_tol=0.0005,
            name=f"{prefix} left runner bears on the front lip",
        )
        ctx.expect_contact(
            panel,
            frame,
            elem_a="side_runner_1",
            elem_b="rear_lip_1",
            contact_tol=0.0005,
            name=f"{prefix} right runner bears on the rear lip",
        )

    _check_panel_captured("lower")

    closed_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 1.0}):
        opened_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap swings outward and upward about the top hinge",
        closed_flap_aabb is not None
        and opened_flap_aabb is not None
        and opened_flap_aabb[0][1] < closed_flap_aabb[0][1] - 0.20
        and opened_flap_aabb[0][2] > closed_flap_aabb[0][2] + 0.08,
        details=f"closed={closed_flap_aabb}, opened={opened_flap_aabb}",
    )

    lower_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_slide: 0.550}):
        upper_panel_aabb = ctx.part_world_aabb(panel)
        _check_panel_captured("raised")
        ctx.expect_gap(
            frame,
            panel,
            axis="z",
            positive_elem="channel_top_stop",
            negative_elem="panel_slab",
            min_gap=0.005,
            max_gap=0.040,
            name="raised security panel remains under the top stop",
        )
    ctx.check(
        "security panel slides vertically upward",
        lower_panel_aabb is not None
        and upper_panel_aabb is not None
        and upper_panel_aabb[0][2] > lower_panel_aabb[0][2] + 0.50,
        details=f"lower={lower_panel_aabb}, upper={upper_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
