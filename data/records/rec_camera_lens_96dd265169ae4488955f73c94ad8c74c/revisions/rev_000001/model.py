from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _lathe_shell(name: str, outer_profile, inner_profile, *, segments: int = 88):
    return _mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ).rotate_y(pi / 2.0),
    )


def _circle_profile(radius: float, *, segments: int = 64):
    return [
        (radius * cos((2.0 * pi * index) / segments), radius * sin((2.0 * pi * index) / segments))
        for index in range(segments)
    ]


def _annulus_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    outer_segments: int = 96,
    inner_segments: int = 72,
):
    return _mesh(
        name,
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=outer_segments),
            [_circle_profile(inner_radius, segments=inner_segments)],
            height=length,
            center=True,
        ).rotate_y(pi / 2.0),
    )


def _bayonet_profile(base_radius: float, lug_radius: float, *, segments: int = 96):
    lug_centers = (pi / 6.0, 5.0 * pi / 6.0, 3.0 * pi / 2.0)
    lug_half_span = 0.23
    points = []
    for index in range(segments):
        angle = (2.0 * pi * index) / segments
        radius = base_radius
        for center_angle in lug_centers:
            wrapped = ((angle - center_angle + pi) % (2.0 * pi)) - pi
            if abs(wrapped) <= lug_half_span:
                blend = 0.5 * (1.0 + cos(pi * wrapped / lug_half_span))
                radius = max(radius, base_radius + (lug_radius - base_radius) * blend)
        points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _grip_ring(
    name: str,
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    ridge_depth: float,
    ridge_count: int,
    segments: int = 88,
):
    z0 = -length * 0.5
    z1 = length * 0.5
    outer = [(outer_radius - ridge_depth * 0.45, z0)]
    samples = max(ridge_count * 2, 8)
    for index in range(samples + 1):
        t = index / samples
        z = z0 + t * length
        ridge_phase = sin(t * ridge_count * 2.0 * pi)
        radius = outer_radius - ridge_depth * (0.5 + 0.5 * ridge_phase)
        if index in (0, samples):
            radius = outer_radius - ridge_depth * 0.45
        outer.append((radius, z))
    inner = [(inner_radius, z0), (inner_radius, z1)]
    return _lathe_shell(name, outer, inner, segments=segments)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_zoom_lens")

    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.16, 0.16, 0.17, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.32, 0.33, 0.35, 1.0))
    mount_steel = model.material("mount_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.22, 0.34, 0.38, 0.58))

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        _annulus_mesh(
            "outer_barrel_rear_shell",
            outer_radius=0.0335,
            inner_radius=0.0275,
            length=0.0300,
        ),
        origin=Origin(xyz=(-0.0290, 0.0, 0.0)),
        material=matte_black,
        name="barrel_shell",
    )
    outer_barrel.visual(
        _annulus_mesh(
            "outer_barrel_zoom_shell",
            outer_radius=0.0366,
            inner_radius=0.0322,
            length=0.0300,
        ),
        origin=Origin(xyz=(-0.0010, 0.0, 0.0)),
        material=matte_black,
        name="zoom_shell",
    )
    outer_barrel.visual(
        _annulus_mesh(
            "outer_barrel_zoom_stop",
            outer_radius=0.0392,
            inner_radius=0.0366,
            length=0.0040,
        ),
        origin=Origin(xyz=(-0.0090, 0.0, 0.0)),
        material=anodized_black,
        name="zoom_stop",
    )
    outer_barrel.inertial = Inertial.from_geometry(
        Box((0.110, 0.080, 0.080)),
        mass=0.55,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    mount_plate = model.part("mount_plate")
    mount_plate.visual(
        _mesh(
            "bayonet_mount_plate",
            ExtrudeWithHolesGeometry(
                _bayonet_profile(0.0305, 0.0335),
                [_circle_profile(0.0220, segments=56)],
                height=0.0048,
                center=True,
            ).rotate_y(pi / 2.0),
        ),
        material=mount_steel,
        name="bayonet_plate",
    )
    mount_plate.visual(
        Cylinder(radius=0.0235, length=0.0040),
        origin=Origin(xyz=(-0.0042, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="rear_throat",
    )
    mount_plate.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0335, length=0.0088),
        mass=0.08,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        _annulus_mesh(
            "zoom_ring_core",
            outer_radius=0.0398,
            inner_radius=0.0374,
            length=0.0300,
        ),
        material=rubber_black,
        name="zoom_grip",
    )
    for index, x_pos in enumerate((-0.0105, -0.0035, 0.0035, 0.0105)):
        zoom_ring.visual(
            _annulus_mesh(
                f"zoom_ring_rib_{index}",
                outer_radius=0.0401,
                inner_radius=0.0388,
                length=0.0046,
            ),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=rubber_black,
        )
    zoom_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0401, length=0.0300),
        mass=0.10,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        _annulus_mesh(
            "inner_barrel_guide_shell",
            outer_radius=0.0322,
            inner_radius=0.0265,
            length=0.0540,
        ),
        origin=Origin(xyz=(0.0260, 0.0, 0.0)),
        material=anodized_black,
        name="telescoping_shell",
    )
    inner_barrel.visual(
        _annulus_mesh(
            "inner_barrel_focus_land",
            outer_radius=0.0348,
            inner_radius=0.0312,
            length=0.0220,
        ),
        origin=Origin(xyz=(0.0500, 0.0, 0.0)),
        material=anodized_black,
        name="focus_land",
    )
    inner_barrel.visual(
        _annulus_mesh(
            "inner_barrel_front_trim",
            outer_radius=0.0298,
            inner_radius=0.0258,
            length=0.0140,
        ),
        origin=Origin(xyz=(0.0600, 0.0, 0.0)),
        material=anodized_black,
        name="front_trim_shell",
    )
    inner_barrel.visual(
        Cylinder(radius=0.0260, length=0.0100),
        origin=Origin(xyz=(0.0710, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=coated_glass,
        name="front_element",
    )
    inner_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0310, length=0.0820),
        mass=0.18,
        origin=Origin(xyz=(0.0410, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _annulus_mesh(
            "focus_ring_core",
            inner_radius=0.0322,
            outer_radius=0.0352,
            length=0.0220,
        ),
        material=rubber_black,
        name="focus_grip",
    )
    for index, x_pos in enumerate((-0.0070, 0.0, 0.0070)):
        focus_ring.visual(
            _annulus_mesh(
                f"focus_ring_rib_{index}",
                outer_radius=0.0353,
                inner_radius=0.0341,
                length=0.0042,
            ),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=rubber_black,
        )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0353, length=0.0220),
        mass=0.06,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "mount_attachment",
        ArticulationType.FIXED,
        parent=outer_barrel,
        child=mount_plate,
        origin=Origin(xyz=(-0.0464, 0.0, 0.0)),
    )
    model.articulation(
        "zoom_ring_rotation",
        ArticulationType.REVOLUTE,
        parent=outer_barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.0080, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "barrel_extension",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.0140, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.08, lower=0.0, upper=0.028),
    )
    model.articulation(
        "focus_ring_rotation",
        ArticulationType.REVOLUTE,
        parent=inner_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0500, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-0.80, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_barrel = object_model.get_part("outer_barrel")
    mount_plate = object_model.get_part("mount_plate")
    zoom_ring = object_model.get_part("zoom_ring")
    inner_barrel = object_model.get_part("inner_barrel")
    focus_ring = object_model.get_part("focus_ring")

    mount_attachment = object_model.get_articulation("mount_attachment")
    zoom_rotation = object_model.get_articulation("zoom_ring_rotation")
    barrel_extension = object_model.get_articulation("barrel_extension")
    focus_rotation = object_model.get_articulation("focus_ring_rotation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        zoom_ring,
        outer_barrel,
        reason="the rubber zoom grip is modeled as a separate rotating sleeve over the zoom carrier to preserve the articulated ring silhouette",
    )
    ctx.allow_overlap(
        inner_barrel,
        outer_barrel,
        reason="the telescoping inner barrel shares a guide sleeve region inside the outer barrel mouth in this simplified visual model",
    )
    ctx.allow_overlap(
        focus_ring,
        inner_barrel,
        reason="the focus grip is represented as a rotating sleeve over the front focusing carrier",
    )

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

    ctx.check("mount_attachment_present", mount_attachment.articulation_type == ArticulationType.FIXED, "bayonet mount should be fixed to the barrel")
    ctx.check("zoom_axis_is_optical", tuple(zoom_rotation.axis) == (1.0, 0.0, 0.0), "zoom ring must rotate around the lens axis")
    ctx.check("extension_axis_is_optical", tuple(barrel_extension.axis) == (1.0, 0.0, 0.0), "inner barrel must telescope along the lens axis")
    ctx.check("focus_axis_is_optical", tuple(focus_rotation.axis) == (1.0, 0.0, 0.0), "focus ring must rotate around the lens axis")

    ctx.expect_contact(mount_plate, outer_barrel, contact_tol=0.0008, name="mount_plate_seats_on_outer_barrel")
    ctx.expect_overlap(mount_plate, outer_barrel, axes="yz", min_overlap=0.050, name="mount_plate_centered_on_barrel")
    ctx.expect_contact(
        zoom_ring,
        outer_barrel,
        elem_a="zoom_grip",
        contact_tol=0.0008,
        name="zoom_ring_is_borne_by_outer_barrel",
    )
    ctx.expect_overlap(zoom_ring, outer_barrel, axes="yz", min_overlap=0.065, name="zoom_ring_concentric_with_barrel")
    ctx.expect_contact(
        inner_barrel,
        outer_barrel,
        elem_a="telescoping_shell",
        elem_b="zoom_shell",
        contact_tol=0.0008,
        name="inner_barrel_guided_in_outer_shell",
    )
    ctx.expect_overlap(inner_barrel, outer_barrel, axes="yz", min_overlap=0.058, name="inner_barrel_concentric_with_outer_barrel")
    ctx.expect_contact(
        focus_ring,
        inner_barrel,
        elem_a="focus_grip",
        elem_b="focus_land",
        contact_tol=0.0008,
        name="focus_ring_is_borne_by_inner_barrel",
    )
    ctx.expect_gap(
        inner_barrel,
        outer_barrel,
        axis="x",
        positive_elem="front_element",
        negative_elem="barrel_shell",
        min_gap=0.020,
        name="front_element_projects_beyond_outer_barrel",
    )

    with ctx.pose({zoom_rotation: 1.20, barrel_extension: 0.028, focus_rotation: 0.55}):
        ctx.expect_contact(
            focus_ring,
            inner_barrel,
            elem_a="focus_grip",
            elem_b="focus_land",
            contact_tol=0.0008,
            name="focus_ring_remains_borne_by_inner_barrel_at_full_zoom",
        )
        ctx.expect_gap(
            inner_barrel,
            outer_barrel,
            axis="x",
            min_gap=0.027,
            max_gap=0.029,
            name="inner_barrel_extends_forward_at_full_zoom",
        )
        ctx.expect_gap(
            focus_ring,
            zoom_ring,
            axis="x",
            min_gap=0.043,
            name="focus_ring_stays_ahead_of_zoom_ring_when_extended",
        )
        ctx.expect_gap(
            inner_barrel,
            outer_barrel,
            axis="x",
            positive_elem="front_element",
            negative_elem="barrel_shell",
            min_gap=0.045,
            name="front_element_advances_when_zoomed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
