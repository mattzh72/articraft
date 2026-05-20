from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


THREAD_PITCH = 0.006
CAP_TWIST_UPPER = 2.35 * pi
CAP_LIFT_PER_RAD = THREAD_PITCH / (2.0 * pi)


def _helix(radius: float, radius_eps: float, pitch: float, height: float, end_relief: float):
    """Rounded-start helical path used for bottle and cap thread crests."""

    turns = height / pitch
    ramp_frac = min(0.16, 0.45 / max(turns, 1.0))

    def path(t: float) -> tuple[float, float, float]:
        if ramp_frac < t < 1.0 - ramp_frac:
            z = height * t
            radial_offset = radius_eps
        elif t <= ramp_frac:
            s = sin(pi / 2.0 * t / ramp_frac)
            z = height * t + end_relief * (s - 1.0)
            radial_offset = radius_eps * s
        else:
            s = sin(pi / 2.0 * (1.0 - t) / ramp_frac)
            z = height * t + end_relief * (1.0 - s)
            radial_offset = radius_eps * s

        r = radius + radial_offset
        angle = 2.0 * pi * turns * t
        return r * cos(angle), r * sin(angle), z

    return path


def _thread_solid(
    *,
    radius: float,
    pitch: float,
    height: float,
    radial_depth: float,
    z_offset: float,
    phase: float = 0.0,
) -> cq.Workplane:
    """Make a shallow, non-overlapping helical thread band."""

    half_width = pitch * 0.16
    relief = pitch * 0.05

    low_root = cq.Workplane("XY").parametricCurve(
        _helix(radius, 0.0, pitch, height, -half_width)
    ).val()
    high_root = cq.Workplane("XY").parametricCurve(
        _helix(radius, 0.0, pitch, height, half_width)
    ).val()
    low_crest = cq.Workplane("XY").parametricCurve(
        _helix(radius, radial_depth, pitch, height, -relief)
    ).val()
    high_crest = cq.Workplane("XY").parametricCurve(
        _helix(radius, radial_depth, pitch, height, relief)
    ).val()

    faces = [
        cq.Face.makeRuledSurface(low_root, high_root),
        cq.Face.makeRuledSurface(low_crest, high_crest),
        cq.Face.makeRuledSurface(low_root, low_crest),
        cq.Face.makeRuledSurface(high_root, high_crest),
    ]
    solid = cq.Solid.makeSolid(cq.Shell.makeShell(faces))
    return (
        cq.Workplane("XY")
        .add(solid)
        .rotate((0, 0, 0), (0, 0, 1), phase)
        .translate((0.0, 0.0, z_offset))
    )


def _bottle_shell() -> cq.Workplane:
    """Compact hollow bottle with a thick desktop-safe base and open mouth."""

    profile = [
        (0.0000, 0.0000),
        (0.0260, 0.0000),
        (0.0310, 0.0030),
        (0.0310, 0.0640),
        (0.0280, 0.0740),
        (0.0200, 0.0870),
        (0.0116, 0.0980),
        (0.0116, 0.1160),
        (0.0134, 0.1182),
        (0.0134, 0.1210),
        (0.0080, 0.1210),
        (0.0080, 0.0995),
        (0.0170, 0.0885),
        (0.0287, 0.0670),
        (0.0287, 0.0060),
        (0.0000, 0.0060),
    ]
    shell = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )
    # Subtle rounding keeps the bottle from reading as a blocky lathe sketch.
    try:
        shell = shell.edges().fillet(0.0007)
    except Exception:
        pass
    return shell


def _cap_shell() -> cq.Workplane:
    """Open-bottom, hollow cap with a closed top and real radial clearance."""

    outer_r = 0.0195
    inner_r = 0.0160
    height = 0.0300
    top_t = 0.0038
    profile = [
        (inner_r, 0.0000),
        (outer_r, 0.0000),
        (outer_r, height),
        (0.0000, height),
        (0.0000, height - top_t),
        (inner_r, height - top_t),
    ]
    shell = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )
    try:
        shell = shell.edges().fillet(0.00045)
    except Exception:
        pass
    return shell


def _thin_band(*, inner_r: float, outer_r: float, height: float, z_offset: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
        .translate((0.0, 0.0, z_offset))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_screwcap_bottle")

    clear_pet = model.material("clear_pet", rgba=(0.72, 0.92, 1.00, 0.36))
    thread_blue = model.material("thread_blue", rgba=(0.20, 0.55, 0.92, 0.82))
    cap_blue = model.material("cap_blue", rgba=(0.05, 0.27, 0.78, 0.48))
    cap_thread = model.material("cap_thread", rgba=(0.95, 0.97, 1.00, 0.92))
    label_white = model.material("label_white", rgba=(0.96, 0.96, 0.90, 0.88))
    ink = model.material("dark_ink", rgba=(0.02, 0.03, 0.05, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_shell(), "bottle_shell", tolerance=0.00035),
        material=clear_pet,
        name="bottle_shell",
    )
    bottle.visual(
        mesh_from_cadquery(
            _thread_solid(
                radius=0.0117,
                pitch=THREAD_PITCH,
                height=0.0170,
                radial_depth=0.0010,
                z_offset=0.1010,
                phase=0.0,
            ),
            "neck_thread",
            tolerance=0.00018,
        ),
        material=thread_blue,
        name="neck_thread",
    )
    bottle.visual(
        mesh_from_cadquery(
            _thin_band(inner_r=0.0303, outer_r=0.0317, height=0.018, z_offset=0.036),
            "desk_label_band",
            tolerance=0.0003,
        ),
        material=label_white,
        name="label_band",
    )
    bottle.visual(
        mesh_from_cadquery(
            _thin_band(inner_r=0.0311, outer_r=0.0320, height=0.003, z_offset=0.043),
            "label_stripe",
            tolerance=0.00025,
        ),
        material=ink,
        name="label_stripe",
    )

    # A zero-geometry screw-axis link lets the cap rotate and translate with the
    # correct helical pitch while keeping the visible cap as one coherent part.
    cap_axis = model.part("cap_axis")

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_cap_shell(), "cap_shell", tolerance=0.00025),
        material=cap_blue,
        name="cap_shell",
    )
    cap.visual(
        mesh_from_cadquery(
            _thin_band(inner_r=0.0083, outer_r=0.0131, height=0.0012, z_offset=0.0250),
            "seal_gasket",
            tolerance=0.00018,
        ),
        material=cap_thread,
        name="seal_gasket",
    )
    for i, z in enumerate((0.006, 0.014, 0.022)):
        cap.visual(
            mesh_from_cadquery(
                _thin_band(inner_r=0.0190, outer_r=0.0204, height=0.0020, z_offset=z),
                f"grip_band_{i}",
                tolerance=0.00025,
            ),
            material=cap_blue,
            name=f"grip_band_{i}",
        )
    cap.visual(
        mesh_from_cadquery(
            _thread_solid(
                radius=0.0160,
                pitch=THREAD_PITCH,
                height=0.0175,
                radial_depth=-0.00295,
                z_offset=0.0048,
                phase=32.0,
            ),
            "cap_thread",
            tolerance=0.00016,
        ),
        material=cap_thread,
        name="cap_thread",
    )

    cap.visual(
        Box((0.0017, 0.0145, 0.0010)),
        origin=Origin(xyz=(0.0075, 0.0, 0.03025), rpy=(0.0, 0.0, pi / 2.0)),
        material=ink,
        name="turn_mark",
    )

    cap_twist = model.articulation(
        "cap_twist",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=cap_axis,
        origin=Origin(xyz=(0.0, 0.0, 0.0960)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CAP_TWIST_UPPER,
            effort=2.0,
            velocity=4.0,
        ),
    )
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=cap_axis,
        child=cap,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CAP_LIFT_PER_RAD * CAP_TWIST_UPPER,
            effort=8.0,
            velocity=0.05,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    cap_twist = object_model.get_articulation("cap_twist")
    cap_lift = object_model.get_articulation("cap_lift")

    ctx.check(
        "cap lift limit follows thread pitch",
        cap_lift.motion_limits is not None
        and cap_twist.motion_limits is not None
        and abs(cap_lift.motion_limits.upper - CAP_LIFT_PER_RAD * cap_twist.motion_limits.upper)
        < 1e-9,
        details=f"twist_limits={cap_twist.motion_limits}, lift_limits={cap_lift.motion_limits}",
    )
    ctx.check(
        "thread axes are coaxial",
        tuple(cap_twist.axis) == (0.0, 0.0, 1.0)
        and tuple(cap_lift.axis) == (0.0, 0.0, 1.0),
        details=f"twist_axis={cap_twist.axis}, lift_axis={cap_lift.axis}",
    )

    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=0.0005,
        name="closed cap is coaxial with neck",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        min_overlap=0.020,
        name="closed cap surrounds threaded neck height",
    )
    ctx.expect_contact(
        cap,
        bottle,
        elem_a="seal_gasket",
        elem_b="bottle_shell",
        contact_tol=0.00005,
        name="closed cap is seated on mouth gasket",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a="cap_thread",
        elem_b="neck_thread",
        min_overlap=0.016,
        name="helical threads share engaged height",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="xy",
        min_overlap=0.016,
        name="cap footprint remains compact over bottle",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_twist: CAP_TWIST_UPPER, cap_lift: CAP_LIFT_PER_RAD * CAP_TWIST_UPPER}):
        ctx.expect_origin_distance(
            cap,
            bottle,
            axes="xy",
            max_dist=0.0005,
            name="loosened cap stays coaxial and stow friendly",
        )
        ctx.expect_overlap(
            cap,
            bottle,
            axes="xy",
            min_overlap=0.016,
            name="loosened cap does not grow footprint",
        )
        raised_pos = ctx.part_world_position(cap)

    ctx.check(
        "twist raises cap by helical pitch",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.0065,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
