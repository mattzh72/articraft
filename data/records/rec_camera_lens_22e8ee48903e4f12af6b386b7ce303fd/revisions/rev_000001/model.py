from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hollow_cylinder(inner_radius: float, outer_radius: float, length: float, name: str):
    """Thin-walled lathed shell aligned to the optical Z axis."""
    half = length / 2.0
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, name)


def _add_radial_box(
    part,
    *,
    name: str,
    radius: float,
    angle: float,
    z: float,
    radial: float,
    tangent: float,
    axial: float,
    material,
    embed: float = 0.0007,
) -> None:
    """Place a small rectangular feature on a cylindrical surface."""
    center_radius = radius + radial / 2.0 - embed
    part.visual(
        Box((radial, tangent, axial)),
        origin=Origin(
            xyz=(center_radius * math.cos(angle), center_radius * math.sin(angle), z),
            rpy=(0.0, 0.0, angle),
        ),
        material=material,
        name=name,
    )


def _add_side_stroke(
    part,
    *,
    name: str,
    radius: float,
    y: float,
    z: float,
    width: float,
    height: float,
    material,
) -> None:
    """Flat raised lettering stroke on the +X side of the cylindrical barrel."""
    radial = 0.0018
    x_surface = math.sqrt(max(radius * radius - y * y, 0.0))
    part.visual(
        Box((radial, width, height)),
        origin=Origin(xyz=(x_surface + radial / 2.0 - 0.0005, y, z)),
        material=material,
        name=name,
    )


def _add_seven_segment_digit(
    part,
    *,
    digit: str,
    prefix: str,
    radius: float,
    y0: float,
    z0: float,
    scale: float,
    material,
) -> None:
    """Approximate lens-barrel numerals with small connected raised strokes."""
    # Segment centers are expressed in a text plane: y is character width,
    # z is optical-axis height on the side of the barrel.
    segs = {
        "0": "abcfed",
        "1": "bc",
        "2": "abged",
        "3": "abgcd",
        "4": "fgbc",
        "5": "afgcd",
        "6": "afgecd",
        "7": "abc",
        "8": "abcdefg",
        "9": "abfgcd",
    }[digit]
    stroke = 0.18 * scale
    long = 0.88 * scale
    half_h = 0.58 * scale
    positions = {
        "a": (0.0, half_h, long, stroke),
        "g": (0.0, 0.0, long, stroke),
        "d": (0.0, -half_h, long, stroke),
        "f": (-long / 2.0, half_h / 2.0, stroke, half_h),
        "b": (long / 2.0, half_h / 2.0, stroke, half_h),
        "e": (-long / 2.0, -half_h / 2.0, stroke, half_h),
        "c": (long / 2.0, -half_h / 2.0, stroke, half_h),
    }
    for seg in segs:
        cy, cz, w, h = positions[seg]
        _add_side_stroke(
            part,
            name=f"{prefix}_{seg}",
            radius=radius,
            y=y0 + cy,
            z=z0 + cz,
            width=w,
            height=h,
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_35mm_t15_prime")

    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.005, 1.0))
    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    rubber = model.material("ribbed_rubber", rgba=(0.012, 0.012, 0.013, 1.0))
    anodized = model.material("dark_anodized", rgba=(0.05, 0.052, 0.055, 1.0))
    white = model.material("engraved_white", rgba=(0.88, 0.86, 0.76, 1.0))
    yellow = model.material("witness_yellow", rgba=(1.0, 0.76, 0.12, 1.0))
    red = model.material("lock_red", rgba=(0.8, 0.02, 0.015, 1.0))
    metal = model.material("brushed_pl_metal", rgba=(0.62, 0.60, 0.55, 1.0))
    glass = model.material("coated_glass", rgba=(0.04, 0.13, 0.18, 0.55))

    body = model.part("lens_body")
    # A continuous hidden optical core keeps the fixed body as one supported
    # assembly while leaving clearance for the rotating rings.
    body.visual(
        Cylinder(radius=0.036, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="core_tube",
    )
    body.visual(
        Cylinder(radius=0.058, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=satin_black,
        name="focus_bearing_rear",
    )
    body.visual(
        Cylinder(radius=0.058, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=satin_black,
        name="focus_bearing_front",
    )
    body.visual(
        Cylinder(radius=0.051, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.068)),
        material=metal,
        name="lock_bearing_rear",
    )
    body.visual(
        Cylinder(radius=0.051, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=metal,
        name="lock_bearing_front",
    )
    body.visual(
        _hollow_cylinder(0.034, 0.052, 0.018, "rear_body_band"),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=satin_black,
        name="rear_body_band",
    )
    body.visual(
        _hollow_cylinder(0.034, 0.056, 0.040, "front_shroud"),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=anodized,
        name="front_shroud",
    )
    body.visual(
        _hollow_cylinder(0.030, 0.049, 0.006, "front_retainer"),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=matte_black,
        name="front_retainer",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=glass,
        name="front_glass",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=model.material("black_coating", rgba=(0.0, 0.0, 0.0, 0.75)),
        name="front_pupil",
    )

    # PL-mount hardware: a metal rear flange with four bayonet ears.
    body.visual(
        _hollow_cylinder(0.018, 0.038, 0.010, "pl_flange"),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=metal,
        name="pl_flange",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _add_radial_box(
            body,
            name=f"pl_lug_{i}",
            radius=0.035,
            angle=angle,
            z=-0.085,
            radial=0.010,
            tangent=0.017,
            axial=0.006,
            material=metal,
            embed=0.0015,
        )
    body.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        material=glass,
        name="rear_element",
    )

    # Raised side label: "35" and a compact T-stop line rendered as lens-barrel
    # strokes instead of a generic texture.
    _add_seven_segment_digit(
        body,
        digit="3",
        prefix="label_3",
        radius=0.056,
        y0=-0.008,
        z0=0.060,
        scale=0.010,
        material=white,
    )
    _add_seven_segment_digit(
        body,
        digit="5",
        prefix="label_5",
        radius=0.056,
        y0=0.004,
        z0=0.060,
        scale=0.010,
        material=white,
    )
    # T1.5 as smaller strokes on the same name band.
    _add_side_stroke(body, name="label_t_top", radius=0.056, y=-0.015, z=0.044, width=0.007, height=0.0018, material=yellow)
    _add_side_stroke(body, name="label_t_stem", radius=0.056, y=-0.015, z=0.040, width=0.0018, height=0.009, material=yellow)
    _add_seven_segment_digit(body, digit="1", prefix="label_1", radius=0.056, y0=-0.006, z0=0.041, scale=0.006, material=yellow)
    _add_side_stroke(body, name="label_dot", radius=0.056, y=0.001, z=0.037, width=0.002, height=0.002, material=yellow)
    _add_seven_segment_digit(body, digit="5", prefix="label_t5", radius=0.056, y0=0.008, z0=0.041, scale=0.006, material=yellow)

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _hollow_cylinder(0.040, 0.058, 0.052, "focus_shell"),
        material=anodized,
        name="focus_shell",
    )
    focus_ring.visual(
        _hollow_cylinder(0.0578, 0.061, 0.014, "focus_mark_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=satin_black,
        name="focus_mark_band",
    )
    for i in range(48):
        angle = 2.0 * math.pi * i / 48.0
        _add_radial_box(
            focus_ring,
            name=f"focus_rib_{i}",
            radius=0.058,
            angle=angle,
            z=-0.008,
            radial=0.004,
            tangent=0.0032,
            axial=0.030,
            material=rubber,
            embed=0.001,
        )
    for i in range(13):
        angle = math.radians(-90 + i * 15)
        long_mark = i % 3 == 0
        _add_radial_box(
            focus_ring,
            name=f"focus_tick_{i}",
            radius=0.061,
            angle=angle,
            z=0.020,
            radial=0.0020,
            tangent=0.0032 if long_mark else 0.0022,
            axial=0.011 if long_mark else 0.006,
            material=white,
            embed=0.0005,
        )
    _add_radial_box(
        focus_ring,
        name="focus_index_mark",
        radius=0.061,
        angle=0.0,
        z=0.020,
        radial=0.0022,
        tangent=0.004,
        axial=0.014,
        material=yellow,
        embed=0.0005,
    )

    pl_lock_ring = model.part("pl_lock_ring")
    pl_lock_ring.visual(
        _hollow_cylinder(0.040, 0.051, 0.018, "pl_lock_shell"),
        material=metal,
        name="pl_lock_shell",
    )
    for i in range(24):
        angle = 2.0 * math.pi * i / 24.0
        _add_radial_box(
            pl_lock_ring,
            name=f"lock_grip_{i}",
            radius=0.051,
            angle=angle,
            z=0.0,
            radial=0.004,
            tangent=0.0045,
            axial=0.014,
            material=satin_black,
            embed=0.001,
        )
    _add_radial_box(
        pl_lock_ring,
        name="lock_dot",
        radius=0.054,
        angle=0.0,
        z=0.004,
        radial=0.0018,
        tangent=0.004,
        axial=0.004,
        material=red,
        embed=0.0003,
    )

    model.articulation(
        "body_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=0.0, upper=4.712),
    )
    model.articulation(
        "body_to_pl_lock_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pl_lock_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=0.785),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lens_body")
    focus_ring = object_model.get_part("focus_ring")
    pl_lock_ring = object_model.get_part("pl_lock_ring")
    focus_joint = object_model.get_articulation("body_to_focus_ring")
    lock_joint = object_model.get_articulation("body_to_pl_lock_ring")

    ctx.check(
        "focus ring is a limited revolute optical-axis joint",
        focus_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(focus_joint.axis) == (0.0, 0.0, 1.0)
        and focus_joint.motion_limits is not None
        and focus_joint.motion_limits.upper >= 4.6,
        details=f"type={focus_joint.articulation_type}, axis={focus_joint.axis}, limits={focus_joint.motion_limits}",
    )
    ctx.check(
        "PL locking ring is a short limited revolute lock",
        lock_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lock_joint.axis) == (0.0, 0.0, 1.0)
        and lock_joint.motion_limits is not None
        and 0.70 <= lock_joint.motion_limits.upper <= 0.90,
        details=f"type={lock_joint.articulation_type}, axis={lock_joint.axis}, limits={lock_joint.motion_limits}",
    )

    # The rings are captured axially between bearing shoulders rather than left
    # floating around the optical core.
    ctx.expect_gap(
        focus_ring,
        body,
        axis="z",
        positive_elem="focus_shell",
        negative_elem="focus_bearing_rear",
        max_gap=0.001,
        max_penetration=0.0,
        name="focus ring seats against rear bearing shoulder",
    )
    ctx.expect_gap(
        body,
        focus_ring,
        axis="z",
        positive_elem="focus_bearing_front",
        negative_elem="focus_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="focus ring seats against front bearing shoulder",
    )
    ctx.expect_gap(
        pl_lock_ring,
        body,
        axis="z",
        positive_elem="pl_lock_shell",
        negative_elem="lock_bearing_rear",
        max_gap=0.001,
        max_penetration=0.0,
        name="PL lock ring seats against rear lock shoulder",
    )
    ctx.expect_gap(
        body,
        pl_lock_ring,
        axis="z",
        positive_elem="lock_bearing_front",
        negative_elem="pl_lock_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="PL lock ring seats against front lock shoulder",
    )
    ctx.expect_within(
        body,
        focus_ring,
        axes="xy",
        inner_elem="core_tube",
        outer_elem="focus_shell",
        margin=0.0,
        name="focus ring is concentric around the optical core",
    )
    ctx.expect_within(
        body,
        pl_lock_ring,
        axes="xy",
        inner_elem="core_tube",
        outer_elem="pl_lock_shell",
        margin=0.0,
        name="PL locking ring is concentric around the optical core",
    )

    def aabb_center(bounds):
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({focus_joint: 0.0, lock_joint: 0.0}):
        rest_focus = aabb_center(ctx.part_element_world_aabb(focus_ring, elem="focus_index_mark"))
        rest_lock = aabb_center(ctx.part_element_world_aabb(pl_lock_ring, elem="lock_dot"))

    with ctx.pose({focus_joint: math.pi / 2.0}):
        turned_focus = aabb_center(ctx.part_element_world_aabb(focus_ring, elem="focus_index_mark"))
    ctx.check(
        "focus index mark orbits around the optical axis",
        rest_focus is not None
        and turned_focus is not None
        and turned_focus[1] > rest_focus[1] + 0.045
        and turned_focus[0] < rest_focus[0] - 0.040,
        details=f"rest={rest_focus}, turned={turned_focus}",
    )

    with ctx.pose({lock_joint: 0.785}):
        turned_lock = aabb_center(ctx.part_element_world_aabb(pl_lock_ring, elem="lock_dot"))
    ctx.check(
        "PL lock dot moves with the locking ring turn",
        rest_lock is not None
        and turned_lock is not None
        and turned_lock[1] > rest_lock[1] + 0.030
        and turned_lock[0] < rest_lock[0] - 0.010,
        details=f"rest={rest_lock}, turned={turned_lock}",
    )

    return ctx.report()


object_model = build_object_model()
