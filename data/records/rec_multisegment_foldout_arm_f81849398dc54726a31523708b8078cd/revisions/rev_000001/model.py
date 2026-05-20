from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _strip_outline(
    dx: float,
    dz: float,
    stations: tuple[float, ...],
    half_widths: tuple[float, ...],
) -> list[tuple[float, float]]:
    length = math.hypot(dx, dz)
    ux = dx / length
    uz = dz / length
    nx = -uz
    nz = ux

    def point_at(station: float, offset: float) -> tuple[float, float]:
        cx = ux * station
        cz = uz * station
        return (cx + nx * offset, cz + nz * offset)

    upper = [point_at(s, w) for s, w in zip(stations, half_widths)]
    lower = [point_at(s, -w) for s, w in reversed(list(zip(stations, half_widths)))]
    return upper + lower


def _pivot_cutter(x: float, z: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(0.08, both=True)


def _slab_from_profile(profile: cq.Workplane, y_min: float, y_max: float) -> cq.Workplane:
    thickness = y_max - y_min
    return profile.extrude(thickness / 2.0, both=True).translate((0.0, 0.5 * (y_min + y_max), 0.0))


def _make_link(
    *,
    dx: float,
    dz: float,
    y_min: float,
    y_max: float,
    root_radius: float,
    distal_radius: float,
    hole_radius: float,
    root_half_width: float,
    mid_half_width_a: float,
    mid_half_width_b: float,
    tip_half_width: float,
    nose_length: float,
) -> cq.Workplane:
    length = math.hypot(dx, dz)
    stations = (
        root_radius * 0.42,
        length * 0.28,
        length * 0.63,
        length + nose_length,
    )
    outline = _strip_outline(
        dx,
        dz,
        stations=stations,
        half_widths=(
            root_half_width,
            mid_half_width_a,
            mid_half_width_b,
            tip_half_width,
        ),
    )

    body = _slab_from_profile(cq.Workplane("XZ").polyline(outline).close(), y_min, y_max)
    root_eye = _slab_from_profile(cq.Workplane("XZ").center(0.0, 0.0).circle(root_radius), y_min, y_max)
    tip_eye = _slab_from_profile(cq.Workplane("XZ").center(dx, dz).circle(distal_radius), y_min, y_max)

    result = body.union(root_eye).union(tip_eye)
    result = result.cut(_pivot_cutter(0.0, 0.0, hole_radius))
    result = result.cut(_pivot_cutter(dx, dz, hole_radius))
    return result


def _make_base_foot(
    *,
    support_y_min: float,
    support_y_max: float,
    pin_y_min: float,
    pin_y_max: float,
    pivot_radius: float,
) -> cq.Workplane:
    pivot_x = 0.020
    pivot_z = 0.145

    foot = cq.Workplane("XY").box(0.250, 0.120, 0.022).translate((0.0, 0.0, 0.011))
    heel = cq.Workplane("XY").box(0.080, 0.120, 0.018).translate((-0.076, 0.0, 0.031))
    toe = cq.Workplane("XY").box(0.054, 0.080, 0.010).translate((0.085, 0.0, 0.027))

    support_outline = [
        (-0.072, 0.022),
        (-0.020, 0.022),
        (0.004, 0.074),
        (0.015, 0.112),
        (pivot_x + 0.012, pivot_z - 0.014),
        (pivot_x + 0.006, pivot_z + 0.016),
        (-0.010, 0.108),
        (-0.055, 0.048),
    ]
    gusset_outline = [
        (-0.058, 0.022),
        (-0.018, 0.022),
        (0.000, 0.068),
        (0.008, 0.094),
        (-0.016, 0.094),
        (-0.048, 0.044),
    ]
    support = _slab_from_profile(
        cq.Workplane("XZ").polyline(support_outline).close(),
        support_y_min,
        support_y_max,
    )
    gusset = _slab_from_profile(
        cq.Workplane("XZ").polyline(gusset_outline).close(),
        support_y_min - 0.004,
        support_y_min + 0.004,
    )
    pivot_pad = _slab_from_profile(
        cq.Workplane("XZ").center(pivot_x, pivot_z).circle(0.023),
        support_y_min,
        support_y_max,
    )
    pivot_pin = _slab_from_profile(
        cq.Workplane("XZ").center(pivot_x, pivot_z).circle(pivot_radius),
        pin_y_min,
        pin_y_max,
    )

    base = foot.union(heel).union(toe).union(support).union(gusset).union(pivot_pad).union(pivot_pin)
    base = base.cut(_pivot_cutter(pivot_x, pivot_z, pivot_radius))
    base = base.union(pivot_pin)
    return base


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_foldout_arm")

    base_mat = model.material("base_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    rocker_mat = model.material("rocker_slate", rgba=(0.27, 0.31, 0.37, 1.0))
    elbow_mat = model.material("elbow_steel", rgba=(0.44, 0.46, 0.49, 1.0))
    forearm_mat = model.material("forearm_alloy", rgba=(0.54, 0.56, 0.58, 1.0))
    tip_mat = model.material("tip_aluminum", rgba=(0.66, 0.67, 0.69, 1.0))

    pivot_hole = 0.0085

    base_foot = model.part("base_foot")
    base_foot.visual(
        mesh_from_cadquery(
            _make_base_foot(
                support_y_min=-0.022,
                support_y_max=-0.010,
                pin_y_min=-0.010,
                pin_y_max=0.022,
                pivot_radius=pivot_hole,
            ),
            "base_foot",
        ),
        material=base_mat,
        name="base_shell",
    )

    rocker = model.part("rocker")
    rocker.visual(
        mesh_from_cadquery(
            _make_link(
                dx=0.140,
                dz=0.032,
                y_min=0.010,
                y_max=0.022,
                root_radius=0.021,
                distal_radius=0.018,
                hole_radius=pivot_hole,
                root_half_width=0.022,
                mid_half_width_a=0.020,
                mid_half_width_b=0.017,
                tip_half_width=0.015,
                nose_length=0.018,
            ),
            "rocker",
        ),
        material=rocker_mat,
        name="rocker_shell",
    )

    elbow_link = model.part("elbow_link")
    elbow_link.visual(
        mesh_from_cadquery(
            _make_link(
                dx=0.170,
                dz=-0.024,
                y_min=0.022,
                y_max=0.034,
                root_radius=0.017,
                distal_radius=0.015,
                hole_radius=0.0075,
                root_half_width=0.019,
                mid_half_width_a=0.014,
                mid_half_width_b=0.016,
                tip_half_width=0.013,
                nose_length=0.016,
            ),
            "elbow_link",
        ),
        material=elbow_mat,
        name="elbow_shell",
    )

    forearm_link = model.part("forearm_link")
    forearm_link.visual(
        mesh_from_cadquery(
            _make_link(
                dx=0.150,
                dz=0.022,
                y_min=0.034,
                y_max=0.046,
                root_radius=0.015,
                distal_radius=0.013,
                hole_radius=0.0065,
                root_half_width=0.016,
                mid_half_width_a=0.013,
                mid_half_width_b=0.015,
                tip_half_width=0.011,
                nose_length=0.015,
            ),
            "forearm_link",
        ),
        material=forearm_mat,
        name="forearm_shell",
    )

    tip_link = model.part("tip_link")
    tip_link.visual(
        mesh_from_cadquery(
            _make_link(
                dx=0.112,
                dz=0.015,
                y_min=0.045,
                y_max=0.055,
                root_radius=0.013,
                distal_radius=0.010,
                hole_radius=0.0055,
                root_half_width=0.014,
                mid_half_width_a=0.011,
                mid_half_width_b=0.009,
                tip_half_width=0.008,
                nose_length=0.014,
            ),
            "tip_link",
        ),
        material=tip_mat,
        name="tip_shell",
    )

    model.articulation(
        "base_to_rocker",
        ArticulationType.REVOLUTE,
        parent=base_foot,
        child=rocker,
        origin=Origin(xyz=(0.020, 0.0, 0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=-0.30,
            upper=1.20,
        ),
    )
    model.articulation(
        "rocker_to_elbow",
        ArticulationType.REVOLUTE,
        parent=rocker,
        child=elbow_link,
        origin=Origin(xyz=(0.140, 0.0, 0.032)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.6,
            lower=-0.95,
            upper=1.05,
        ),
    )
    model.articulation(
        "elbow_to_forearm",
        ArticulationType.REVOLUTE,
        parent=elbow_link,
        child=forearm_link,
        origin=Origin(xyz=(0.170, 0.0, -0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-1.00,
            upper=0.95,
        ),
    )
    model.articulation(
        "forearm_to_tip",
        ArticulationType.REVOLUTE,
        parent=forearm_link,
        child=tip_link,
        origin=Origin(xyz=(0.150, 0.0, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.80,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_foot = object_model.get_part("base_foot")
    rocker = object_model.get_part("rocker")
    elbow_link = object_model.get_part("elbow_link")
    forearm_link = object_model.get_part("forearm_link")
    tip_link = object_model.get_part("tip_link")

    base_to_rocker = object_model.get_articulation("base_to_rocker")
    rocker_to_elbow = object_model.get_articulation("rocker_to_elbow")
    elbow_to_forearm = object_model.get_articulation("elbow_to_forearm")
    forearm_to_tip = object_model.get_articulation("forearm_to_tip")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        base_foot,
        rocker,
        reason="The grounded base includes an integral pivot pin that passes through the rocker eye as the first revolute bearing.",
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

    ctx.check(
        "expected serial arm parts exist",
        all(
            part is not None
            for part in (base_foot, rocker, elbow_link, forearm_link, tip_link)
        ),
        "One or more expected parts are missing.",
    )
    ctx.check(
        "all arm joints share one motion axis",
        all(
            articulation.axis == (0.0, -1.0, 0.0)
            for articulation in (
                base_to_rocker,
                rocker_to_elbow,
                elbow_to_forearm,
                forearm_to_tip,
            )
        ),
        "All four revolute joints should rotate in the same XZ motion plane.",
    )

    ctx.expect_contact(base_foot, rocker, name="base clevis captures rocker")
    ctx.expect_contact(rocker, elbow_link, name="rocker clevis captures elbow link")
    ctx.expect_contact(elbow_link, forearm_link, name="elbow clevis captures forearm")
    ctx.expect_contact(forearm_link, tip_link, name="forearm clevis captures tip link")

    with ctx.pose(
        {
            base_to_rocker: 0.45,
            rocker_to_elbow: 0.35,
            elbow_to_forearm: 0.25,
            forearm_to_tip: 0.15,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in deployed pose")

        positions = [
            ctx.part_world_position(part)
            for part in (rocker, elbow_link, forearm_link, tip_link)
        ]
        same_plane = all(position is not None and abs(position[1]) < 1e-6 for position in positions)
        tip_position = positions[-1]
        reach_ok = tip_position is not None and tip_position[0] > 0.20 and tip_position[2] > 0.34

        ctx.check(
            "deployed chain stays in one motion plane",
            same_plane,
            "A revolute axis appears misaligned because the arm leaves the shared XZ plane.",
        )
        ctx.check(
            "deployed chain reaches up and forward",
            reach_ok,
            "The tip should swing above the base and remain meaningfully forward in a deployed pose.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
