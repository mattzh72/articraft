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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _shift_profile(
    profile: list[tuple[float, float]], dx: float = 0.0, dy: float = 0.0
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _xy_section(
    *,
    length: float,
    width: float,
    z: float,
    radius: float,
    x_shift: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_shift, y, z)
        for x, y in rounded_rect_profile(
            length, width, radius, corner_segments=corner_segments
        )
    ]


def _framed_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    hole_profiles: list[list[tuple[float, float]]],
    radius: float = 0.022,
):
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        hole_profiles,
        thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _build_top_ring_mesh():
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.48, 0.48, 0.040, corner_segments=8),
        [rounded_rect_profile(0.34, 0.34, 0.020, corner_segments=8)],
        0.022,
        center=True,
    )


def _build_top_liner_mesh():
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.36, 0.36, 0.025, corner_segments=8),
        [rounded_rect_profile(0.30, 0.30, 0.018, corner_segments=8)],
        0.012,
        center=True,
    )


def _build_flap_shell_mesh():
    return section_loft(
        [
            _xy_section(length=0.50, width=0.48, z=-0.022, radius=0.040),
            _xy_section(length=0.488, width=0.468, z=-0.004, radius=0.036, x_shift=0.004),
            _xy_section(length=0.474, width=0.454, z=0.016, radius=0.032, x_shift=0.008),
        ]
    )


def _build_flap_inner_ring_mesh():
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.43, 0.41, 0.027, corner_segments=8),
        [rounded_rect_profile(0.31, 0.29, 0.018, corner_segments=8)],
        0.014,
        center=True,
    )


def _build_gasket_ring_mesh():
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.372, 0.372, 0.020, corner_segments=8),
        [rounded_rect_profile(0.320, 0.320, 0.014, corner_segments=8)],
        0.004,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_roof_vent_tower")

    painted_metal = model.material("painted_metal", rgba=(0.80, 0.82, 0.84, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.47, 0.50, 0.54, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    elastomer = model.material("elastomer", rgba=(0.07, 0.07, 0.08, 1.0))

    front_panel_mesh = _framed_panel_mesh(
        width=0.40,
        height=0.36,
        thickness=0.018,
        hole_profiles=[
            _shift_profile(rounded_rect_profile(0.12, 0.20, 0.014, corner_segments=6), -0.088),
            _shift_profile(rounded_rect_profile(0.12, 0.20, 0.014, corner_segments=6), 0.088),
        ],
    )
    side_panel_mesh = _framed_panel_mesh(
        width=0.34,
        height=0.36,
        thickness=0.018,
        hole_profiles=[
            rounded_rect_profile(0.18, 0.22, 0.016, corner_segments=6),
        ],
    ).rotate_z(math.pi / 2.0)

    tower_body = model.part("tower_body")
    tower_body.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.62, 0.62, 0.070, corner_segments=10),
                0.08,
            ),
            "vent_base_plinth",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_metal,
        name="base_plinth",
    )
    tower_body.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.54, 0.54, 0.055, corner_segments=10),
                0.09,
            ),
            "vent_transition_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=painted_metal,
        name="transition_collar",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower_body.visual(
                Box((0.055, 0.055, 0.36)),
                origin=Origin(xyz=(0.22 * x_sign, 0.22 * y_sign, 0.35)),
                material=painted_metal,
                name=f"corner_post_{'r' if x_sign > 0 else 'l'}_{'f' if y_sign > 0 else 'r'}",
            )

    tower_body.visual(
        mesh_from_geometry(front_panel_mesh, "vent_front_panel"),
        origin=Origin(xyz=(0.0, 0.231, 0.35)),
        material=painted_metal,
        name="front_panel",
    )
    tower_body.visual(
        mesh_from_geometry(front_panel_mesh.copy().rotate_z(math.pi), "vent_rear_panel"),
        origin=Origin(xyz=(0.0, -0.231, 0.35)),
        material=painted_metal,
        name="rear_panel",
    )
    tower_body.visual(
        mesh_from_geometry(side_panel_mesh, "vent_right_panel"),
        origin=Origin(xyz=(0.231, 0.0, 0.35)),
        material=painted_metal,
        name="right_panel",
    )
    tower_body.visual(
        mesh_from_geometry(side_panel_mesh.copy().rotate_z(math.pi), "vent_left_panel"),
        origin=Origin(xyz=(-0.231, 0.0, 0.35)),
        material=painted_metal,
        name="left_panel",
    )

    tower_body.visual(
        mesh_from_geometry(_build_top_ring_mesh(), "vent_top_frame"),
        origin=Origin(xyz=(0.0, 0.0, 0.537)),
        material=painted_metal,
        name="top_frame",
    )
    tower_body.visual(
        mesh_from_geometry(_build_top_liner_mesh(), "vent_top_liner"),
        origin=Origin(xyz=(0.0, 0.0, 0.528)),
        material=polymer_dark,
        name="top_liner",
    )
    tower_body.visual(
        Box((0.10, 0.56, 0.016)),
        origin=Origin(xyz=(-0.19, 0.0, 0.540)),
        material=painted_metal,
        name="rear_hinge_bridge",
    )

    for side, y_pos in (("left", -0.266), ("right", 0.266)):
        tower_body.visual(
            Box((0.028, 0.040, 0.038)),
            origin=Origin(xyz=(-0.223, y_pos, 0.567)),
            material=hinge_metal,
            name=f"{side}_hinge_plate",
        )
        tower_body.visual(
            Cylinder(radius=0.014, length=0.030),
            origin=Origin(
                xyz=(-0.223, y_pos, 0.574),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"{side}_hinge_barrel",
        )

    for side, y_pos in (("left", -0.275), ("right", 0.275)):
        tower_body.visual(
            Box((0.070, 0.018, 0.018)),
            origin=Origin(xyz=(-0.175, y_pos, 0.547)),
            material=hinge_metal,
            name=f"{side}_stop_brace",
        )
        tower_body.visual(
            Box((0.024, 0.028, 0.052)),
            origin=Origin(xyz=(-0.150, y_pos, 0.581)),
            material=hinge_metal,
            name=f"{side}_stop_lug",
        )

    tower_body.inertial = Inertial.from_geometry(
        Box((0.62, 0.62, 0.60)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
    )

    weather_flap = model.part("weather_flap")
    weather_flap.visual(
        mesh_from_geometry(_build_flap_shell_mesh(), "vent_flap_shell"),
        origin=Origin(xyz=(0.23, 0.0, -0.001)),
        material=painted_metal,
        name="flap_shell",
    )
    weather_flap.visual(
        mesh_from_geometry(_build_flap_inner_ring_mesh(), "vent_flap_inner_ring"),
        origin=Origin(xyz=(0.23, 0.0, -0.014)),
        material=polymer_dark,
        name="flap_inner_ring",
    )
    weather_flap.visual(
        mesh_from_geometry(_build_gasket_ring_mesh(), "vent_flap_gasket"),
        origin=Origin(xyz=(0.23, 0.0, -0.024)),
        material=elastomer,
        name="gasket_ring",
    )
    weather_flap.visual(
        Box((0.020, 0.320, 0.016)),
        origin=Origin(xyz=(0.470, 0.0, -0.016)),
        material=painted_metal,
        name="front_drip_lip",
    )
    weather_flap.visual(
        Box((0.080, 0.180, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, -0.010)),
        material=hinge_metal,
        name="center_hinge_strap",
    )
    for side, y_pos in (("left", -0.225), ("right", 0.225)):
        weather_flap.visual(
            Box((0.088, 0.050, 0.010)),
            origin=Origin(xyz=(0.040, y_pos, -0.015)),
            material=hinge_metal,
            name=f"{side}_hinge_ear",
        )
        weather_flap.visual(
            Cylinder(radius=0.014, length=0.028),
            origin=Origin(
                xyz=(0.0, y_pos, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"{side}_flap_hinge_barrel",
        )
        weather_flap.visual(
            Box((0.100, 0.018, 0.012)),
            origin=Origin(
                xyz=(0.090, -0.245 if side == "left" else 0.245, -0.012)
            ),
            material=hinge_metal,
            name=f"{side}_stop_arm",
        )
        weather_flap.visual(
            Box((0.024, 0.028, 0.014)),
            origin=Origin(
                xyz=(0.142, -0.245 if side == "left" else 0.245, -0.008)
            ),
            material=hinge_metal,
            name=f"{side}_stop_pad",
        )

    weather_flap.inertial = Inertial.from_geometry(
        Box((0.50, 0.48, 0.05)),
        mass=2.2,
        origin=Origin(xyz=(0.23, 0.0, -0.005)),
    )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower_body,
        child=weather_flap,
        origin=Origin(xyz=(-0.223, 0.0, 0.574)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_body = object_model.get_part("tower_body")
    weather_flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("body_to_flap")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    body_visuals = {visual.name for visual in tower_body.visuals}
    flap_visuals = {visual.name for visual in weather_flap.visuals}
    ctx.check(
        "explicit_hinge_hardware_present",
        {
            "left_hinge_plate",
            "right_hinge_plate",
            "left_hinge_barrel",
            "right_hinge_barrel",
            "left_stop_lug",
            "right_stop_lug",
        }.issubset(body_visuals)
        and {
            "center_hinge_strap",
            "left_hinge_ear",
            "right_hinge_ear",
            "left_flap_hinge_barrel",
            "right_flap_hinge_barrel",
            "left_stop_pad",
            "right_stop_pad",
        }.issubset(flap_visuals),
        "Missing one or more explicit hinge plates, barrels, or stop features.",
    )
    ctx.check(
        "hinge_axis_opens_upward",
        hinge.axis == (0.0, -1.0, 0.0)
        and hinge.motion_limits is not None
        and math.isclose(hinge.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and math.isclose(hinge.motion_limits.upper or 0.0, 1.15, abs_tol=1e-9),
        f"Unexpected hinge axis or limits: axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            weather_flap,
            tower_body,
            axis="z",
            positive_elem="gasket_ring",
            negative_elem="top_frame",
            min_gap=-0.0005,
            max_gap=0.0015,
            name="gasket_ring_seats_on_top_frame",
        )
        ctx.expect_within(
            tower_body,
            weather_flap,
            axes="xy",
            inner_elem="top_frame",
            outer_elem="flap_shell",
            margin=0.010,
            name="flap_footprint_covers_top_frame",
        )
        ctx.expect_contact(
            weather_flap,
            tower_body,
            contact_tol=0.002,
            name="flap_contacts_body_when_closed",
        )

    closed_flap_shell = ctx.part_element_world_aabb(weather_flap, elem="flap_shell")
    upper_limit = hinge.motion_limits.upper if hinge.motion_limits is not None else 0.0
    with ctx.pose({hinge: upper_limit}):
        ctx.expect_gap(
            weather_flap,
            tower_body,
            axis="z",
            positive_elem="front_drip_lip",
            negative_elem="top_frame",
            min_gap=0.11,
            name="front_edge_lifts_clear_when_open",
        )
        open_flap_shell = ctx.part_element_world_aabb(weather_flap, elem="flap_shell")

    if closed_flap_shell is None or open_flap_shell is None:
        ctx.fail("flap_shell_aabb_available", "Could not evaluate flap shell world AABBs.")
    else:
        closed_top_z = closed_flap_shell[1][2]
        open_top_z = open_flap_shell[1][2]
        ctx.check(
            "flap_opens_significantly",
            open_top_z > closed_top_z + 0.15,
            f"Closed top z={closed_top_z:.4f}, open top z={open_top_z:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
