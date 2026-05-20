from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _beam_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """Return midpoint, length, and Y-pitch for a box whose local +X follows start->end in the XZ plane."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dz = ez - sz
    length = math.sqrt(dx * dx + dz * dz)
    pitch = -math.atan2(dz, dx)
    return ((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5, length, pitch)


def _add_beam(part, name: str, start, end, width_y: float, depth_z: float, material) -> None:
    cx, cy, cz, length, pitch = _beam_origin(start, end)
    part.visual(
        Box((length, width_y, depth_z)),
        origin=Origin(xyz=(cx, cy, cz), rpy=(0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _saddle_mesh():
    # Single, shallow, injection-foam saddle shell: wider rear, tapered nose, rounded vertical edges.
    return (
        cq.Workplane("XY")
        .polyline(
            [
                (-0.135, -0.090),
                (-0.030, -0.105),
                (0.130, -0.042),
                (0.150, 0.000),
                (0.130, 0.042),
                (-0.030, 0.105),
                (-0.135, 0.090),
            ]
        )
        .close()
        .extrude(0.045)
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .fillet(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_stationary_bike")

    powder = model.material("powder_coated_steel", rgba=(0.06, 0.07, 0.075, 1.0))
    dark = model.material("matte_black_plastic", rgba=(0.015, 0.016, 0.017, 1.0))
    graphite = model.material("graphite_housing", rgba=(0.18, 0.19, 0.20, 1.0))
    seam = model.material("molded_seam_shadow", rgba=(0.02, 0.02, 0.022, 1.0))
    steel = model.material("brushed_bearing_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.03, 0.03, 0.028, 1.0))
    foam = model.material("textured_saddle_foam", rgba=(0.025, 0.025, 0.023, 1.0))
    warning = model.material("assembly_mark_yellow", rgba=(0.95, 0.68, 0.08, 1.0))

    frame = model.part("frame")

    # Low part-count base: two rectangular extrusions and two cross stampings with molded feet.
    frame.visual(Box((1.20, 0.045, 0.060)), origin=Origin(xyz=(0.00, -0.220, 0.050)), material=powder, name="base_rail_0")
    frame.visual(Box((1.20, 0.045, 0.060)), origin=Origin(xyz=(0.00, 0.220, 0.050)), material=powder, name="base_rail_1")
    frame.visual(Box((0.095, 0.620, 0.060)), origin=Origin(xyz=(-0.545, 0.000, 0.050)), material=powder, name="front_cross_foot")
    frame.visual(Box((0.095, 0.620, 0.060)), origin=Origin(xyz=(0.545, 0.000, 0.050)), material=powder, name="rear_cross_foot")
    for i, (x, y) in enumerate(((-0.545, -0.335), (-0.545, 0.335), (0.545, -0.335), (0.545, 0.335))):
        frame.visual(Box((0.120, 0.055, 0.026)), origin=Origin(xyz=(x, y, 0.023)), material=rubber, name=f"rubber_foot_{i}")

    # Welded/stamped main frame, kept to straight cuts and simple rectangular tubes.
    _add_beam(frame, "main_tube", (0.500, 0.0, 0.085), (-0.145, 0.0, 0.620), 0.070, 0.082, powder)
    _add_beam(frame, "front_upright", (-0.515, 0.0, 0.080), (-0.535, 0.0, 0.720), 0.070, 0.075, powder)
    _add_beam(frame, "seat_strut", (0.485, 0.0, 0.080), (0.205, 0.0, 0.540), 0.060, 0.070, powder)
    _add_beam(frame, "housing_bridge", (-0.545, 0.0, 0.210), (-0.210, 0.0, 0.315), 0.055, 0.060, powder)
    frame.visual(Box((0.180, 0.095, 0.030)), origin=Origin(xyz=(-0.245, 0.0, 0.158)), material=powder, name="housing_mount_plate")

    # Molded two-piece flywheel cover, with a central bearing boss and visible snap/bolt bosses.
    frame.visual(
        Cylinder(radius=0.275, length=0.150),
        origin=Origin(xyz=(-0.250, 0.000, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="flywheel_cover",
    )
    frame.visual(
        Cylinder(radius=0.279, length=0.012),
        origin=Origin(xyz=(-0.250, -0.082, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seam,
        name="cover_seam_0",
    )
    frame.visual(
        Cylinder(radius=0.279, length=0.012),
        origin=Origin(xyz=(-0.250, 0.082, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seam,
        name="cover_seam_1",
    )
    frame.visual(
        Cylinder(radius=0.062, length=0.190),
        origin=Origin(xyz=(-0.250, 0.000, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="bearing_boss",
    )
    frame.visual(
        Cylinder(radius=0.036, length=0.205),
        origin=Origin(xyz=(-0.250, 0.000, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bearing_insert",
    )
    for i, (dx, dz) in enumerate(((0.0, 0.215), (-0.185, 0.095), (-0.170, -0.120), (0.155, -0.155))):
        frame.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(-0.250 + dx, 0.088, 0.430 + dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=seam,
            name=f"cover_screw_boss_{i}",
        )

    # Hollow square saddle sleeve and a thick clamp collar; inner post is clearanced but retained.
    sleeve_h = 0.320
    sleeve_z = 0.680
    sleeve_x = 0.205
    od = 0.064
    wall = 0.008
    frame.visual(Box((wall, od, sleeve_h)), origin=Origin(xyz=(sleeve_x - od / 2 + wall / 2, 0.0, sleeve_z)), material=powder, name="saddle_sleeve_wall_x0")
    frame.visual(Box((wall, od, sleeve_h)), origin=Origin(xyz=(sleeve_x + od / 2 - wall / 2, 0.0, sleeve_z)), material=powder, name="saddle_sleeve_wall_x1")
    frame.visual(Box((od, wall, sleeve_h)), origin=Origin(xyz=(sleeve_x, -od / 2 + wall / 2, sleeve_z)), material=powder, name="saddle_sleeve_wall_y0")
    frame.visual(Box((od, wall, sleeve_h)), origin=Origin(xyz=(sleeve_x, od / 2 - wall / 2, sleeve_z)), material=powder, name="saddle_sleeve_wall_y1")
    collar_od = 0.100
    collar_id = 0.050
    collar_wall = 0.012
    collar_h = 0.075
    # Four-piece visual collar leaves the telescoping post's square clearance hole visible.
    frame.visual(Box((collar_wall, collar_od, collar_h)), origin=Origin(xyz=(sleeve_x - collar_id / 2 - collar_wall / 2, 0.0, 0.800)), material=powder, name="saddle_collar_wall_x0")
    frame.visual(Box((collar_wall, collar_od, collar_h)), origin=Origin(xyz=(sleeve_x + collar_id / 2 + collar_wall / 2, 0.0, 0.800)), material=powder, name="saddle_collar_wall_x1")
    frame.visual(Box((collar_od, collar_wall, collar_h)), origin=Origin(xyz=(sleeve_x, -collar_id / 2 - collar_wall / 2, 0.800)), material=powder, name="saddle_collar_wall_y0")
    frame.visual(Box((collar_od, collar_wall, collar_h)), origin=Origin(xyz=(sleeve_x, collar_id / 2 + collar_wall / 2, 0.800)), material=powder, name="saddle_collar_wall_y1")
    frame.visual(
        Cylinder(radius=0.020, length=0.046),
        origin=Origin(xyz=(sleeve_x, 0.067, 0.795), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder,
        name="saddle_clamp_boss",
    )
    frame.visual(Box((0.080, 0.012, 0.024)), origin=Origin(xyz=(sleeve_x, 0.046, 0.832)), material=powder, name="saddle_pad_upper")
    frame.visual(Box((0.080, 0.012, 0.024)), origin=Origin(xyz=(sleeve_x, 0.046, 0.758)), material=powder, name="saddle_pad_lower")
    frame.visual(Box((0.010, 0.014, 0.052)), origin=Origin(xyz=(sleeve_x - 0.016, 0.046, 0.795)), material=powder, name="saddle_boss_web_0")
    frame.visual(Box((0.010, 0.014, 0.052)), origin=Origin(xyz=(sleeve_x + 0.016, 0.046, 0.795)), material=powder, name="saddle_boss_web_1")

    # Hollow square handlebar sleeve with matching collar and clamp boss.
    bar_x = -0.535
    bar_z = 0.865
    bar_h = 0.300
    frame.visual(Box((wall, od, bar_h)), origin=Origin(xyz=(bar_x - od / 2 + wall / 2, 0.0, bar_z)), material=powder, name="bar_sleeve_wall_x0")
    frame.visual(Box((wall, od, bar_h)), origin=Origin(xyz=(bar_x + od / 2 - wall / 2, 0.0, bar_z)), material=powder, name="bar_sleeve_wall_x1")
    frame.visual(Box((od, wall, bar_h)), origin=Origin(xyz=(bar_x, -od / 2 + wall / 2, bar_z)), material=powder, name="bar_sleeve_wall_y0")
    frame.visual(Box((od, wall, bar_h)), origin=Origin(xyz=(bar_x, od / 2 - wall / 2, bar_z)), material=powder, name="bar_sleeve_wall_y1")
    frame.visual(Box((collar_wall, collar_od, collar_h)), origin=Origin(xyz=(bar_x - collar_id / 2 - collar_wall / 2, 0.0, 0.985)), material=powder, name="bar_collar_wall_x0")
    frame.visual(Box((collar_wall, collar_od, collar_h)), origin=Origin(xyz=(bar_x + collar_id / 2 + collar_wall / 2, 0.0, 0.985)), material=powder, name="bar_collar_wall_x1")
    frame.visual(Box((collar_od, collar_wall, collar_h)), origin=Origin(xyz=(bar_x, -collar_id / 2 - collar_wall / 2, 0.985)), material=powder, name="bar_collar_wall_y0")
    frame.visual(Box((collar_od, collar_wall, collar_h)), origin=Origin(xyz=(bar_x, collar_id / 2 + collar_wall / 2, 0.985)), material=powder, name="bar_collar_wall_y1")
    frame.visual(
        Cylinder(radius=0.020, length=0.046),
        origin=Origin(xyz=(bar_x, 0.067, 0.985), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder,
        name="bar_clamp_boss",
    )
    frame.visual(Box((0.080, 0.012, 0.024)), origin=Origin(xyz=(bar_x, 0.046, 1.017)), material=powder, name="bar_pad_upper")
    frame.visual(Box((0.080, 0.012, 0.024)), origin=Origin(xyz=(bar_x, 0.046, 0.943)), material=powder, name="bar_pad_lower")
    frame.visual(Box((0.010, 0.014, 0.052)), origin=Origin(xyz=(bar_x - 0.016, 0.046, 0.985)), material=powder, name="bar_boss_web_0")
    frame.visual(Box((0.010, 0.014, 0.052)), origin=Origin(xyz=(bar_x + 0.016, 0.046, 0.985)), material=powder, name="bar_boss_web_1")

    # One top-loaded resistance dial boss keeps the cable and threads inside the molded cover.
    frame.visual(Cylinder(radius=0.042, length=0.035), origin=Origin(xyz=(-0.250, 0.0, 0.705)), material=graphite, name="resistance_boss")
    frame.visual(Box((0.008, 0.006, 0.005)), origin=Origin(xyz=(-0.250, 0.040, 0.725)), material=warning, name="resistance_index_mark")

    # Height-adjusting saddle post, one welded saddle rail, one molded saddle foam pad.
    saddle_post = model.part("saddle_post")
    saddle_post.visual(Box((0.040, 0.040, 0.620)), origin=Origin(xyz=(0.0, 0.0, 0.070)), material=powder, name="inner_post")
    saddle_post.visual(Box((0.030, 0.005, 0.080)), origin=Origin(xyz=(0.0, 0.0215, -0.160)), material=dark, name="glide_pad_0")
    saddle_post.visual(Box((0.030, 0.005, 0.080)), origin=Origin(xyz=(0.0, -0.0215, -0.160)), material=dark, name="glide_pad_1")
    saddle_post.visual(Box((0.190, 0.050, 0.026)), origin=Origin(xyz=(0.070, 0.0, 0.365)), material=powder, name="saddle_rail")
    saddle_post.visual(Box((0.060, 0.040, 0.090)), origin=Origin(xyz=(0.020, 0.0, 0.335)), material=powder, name="rail_riser")
    saddle_post.visual(
        mesh_from_cadquery(_saddle_mesh(), "saddle_pad"),
        origin=Origin(xyz=(0.075, 0.0, 0.378)),
        material=foam,
        name="saddle_pad",
    )

    # Handlebar height post with integral crossbar and overmolded grips.
    handle_post = model.part("handle_post")
    handle_post.visual(Box((0.040, 0.040, 0.560)), origin=Origin(xyz=(0.0, 0.0, 0.035)), material=powder, name="inner_post")
    handle_post.visual(Box((0.030, 0.005, 0.080)), origin=Origin(xyz=(0.0, 0.0215, -0.160)), material=dark, name="glide_pad_0")
    handle_post.visual(Box((0.030, 0.005, 0.080)), origin=Origin(xyz=(0.0, -0.0215, -0.160)), material=dark, name="glide_pad_1")
    handle_post.visual(Box((0.060, 0.050, 0.130)), origin=Origin(xyz=(0.000, 0.000, 0.335)), material=powder, name="bar_stem")
    handle_post.visual(
        Cylinder(radius=0.018, length=0.620),
        origin=Origin(xyz=(0.000, 0.000, 0.405), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder,
        name="handlebar",
    )
    handle_post.visual(
        Cylinder(radius=0.023, length=0.150),
        origin=Origin(xyz=(0.000, -0.315, 0.405), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip_0",
    )
    handle_post.visual(
        Cylinder(radius=0.023, length=0.150),
        origin=Origin(xyz=(0.000, 0.315, 0.405), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip_1",
    )

    # Crank assembly: one axle, two stamped arms, and two pedal bosses.
    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.014, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_shaft",
    )
    crank.visual(Cylinder(radius=0.045, length=0.034), origin=Origin(xyz=(0.0, 0.118, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=steel, name="right_hub")
    crank.visual(Cylinder(radius=0.045, length=0.034), origin=Origin(xyz=(0.0, -0.118, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=steel, name="left_hub")
    crank.visual(Box((0.036, 0.018, 0.210)), origin=Origin(xyz=(0.0, 0.119, -0.105)), material=steel, name="right_arm")
    crank.visual(Box((0.036, 0.018, 0.210)), origin=Origin(xyz=(0.0, -0.119, 0.105)), material=steel, name="left_arm")
    crank.visual(Cylinder(radius=0.025, length=0.028), origin=Origin(xyz=(0.0, 0.139, -0.210), rpy=(math.pi / 2.0, 0.0, 0.0)), material=steel, name="right_pedal_boss")
    crank.visual(Cylinder(radius=0.025, length=0.028), origin=Origin(xyz=(0.0, -0.139, 0.210), rpy=(math.pi / 2.0, 0.0, 0.0)), material=steel, name="left_pedal_boss")

    right_pedal = model.part("right_pedal")
    right_pedal.visual(Cylinder(radius=0.006, length=0.090), origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=steel, name="spindle")
    right_pedal.visual(Box((0.120, 0.060, 0.030)), origin=Origin(xyz=(0.0, 0.060, 0.0)), material=dark, name="platform")
    right_pedal.visual(Box((0.110, 0.016, 0.010)), origin=Origin(xyz=(0.0, 0.060, 0.020)), material=seam, name="tread_bar")

    left_pedal = model.part("left_pedal")
    left_pedal.visual(Cylinder(radius=0.006, length=0.090), origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=steel, name="spindle")
    left_pedal.visual(Box((0.120, 0.060, 0.030)), origin=Origin(xyz=(0.0, -0.060, 0.0)), material=dark, name="platform")
    left_pedal.visual(Box((0.110, 0.016, 0.010)), origin=Origin(xyz=(0.0, -0.060, 0.020)), material=seam, name="tread_bar")

    # Two identical low-cost clamp knobs; their screw shanks intentionally sit in the clamp bosses.
    def add_clamp_knob(part) -> None:
        part.visual(Cylinder(radius=0.006, length=0.050), origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=steel, name="screw_shank")
        part.visual(Cylinder(radius=0.030, length=0.026), origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark, name="knob_cap")
        for i in range(6):
            a = i * math.tau / 6.0
            part.visual(
                Box((0.014, 0.030, 0.010)),
                origin=Origin(xyz=(0.034 * math.cos(a), 0.013, 0.034 * math.sin(a)), rpy=(0.0, -a, 0.0)),
                material=dark,
                name=f"grip_lobe_{i}",
            )

    saddle_knob = model.part("saddle_knob")
    add_clamp_knob(saddle_knob)

    bar_knob = model.part("bar_knob")
    add_clamp_knob(bar_knob)

    resistance_knob = model.part("resistance_knob")
    resistance_knob.visual(Cylinder(radius=0.036, length=0.030), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=dark, name="dial_grip")
    resistance_knob.visual(Box((0.010, 0.050, 0.006)), origin=Origin(xyz=(0.0, 0.0, 0.031)), material=warning, name="dial_pointer")

    model.articulation(
        "frame_to_saddle_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle_post,
        origin=Origin(xyz=(sleeve_x, 0.0, sleeve_z + sleeve_h / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.160),
    )
    model.articulation(
        "frame_to_handle_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handle_post,
        origin=Origin(xyz=(bar_x, 0.0, bar_z + bar_h / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.18, lower=0.0, upper=0.140),
    )
    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(-0.250, 0.0, 0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=8.0),
    )
    model.articulation(
        "crank_to_right_pedal",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=right_pedal,
        origin=Origin(xyz=(0.0, 0.158, -0.210)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "crank_to_left_pedal",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=left_pedal,
        origin=Origin(xyz=(0.0, -0.158, 0.210)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "frame_to_saddle_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=saddle_knob,
        origin=Origin(xyz=(sleeve_x, 0.089, 0.795)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )
    model.articulation(
        "frame_to_bar_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=bar_knob,
        origin=Origin(xyz=(bar_x, 0.089, 0.985)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )
    model.articulation(
        "frame_to_resistance_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=resistance_knob,
        origin=Origin(xyz=(-0.250, 0.0, 0.7225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crank = object_model.get_part("crank")
    saddle_post = object_model.get_part("saddle_post")
    handle_post = object_model.get_part("handle_post")
    saddle_knob = object_model.get_part("saddle_knob")
    bar_knob = object_model.get_part("bar_knob")
    resistance_knob = object_model.get_part("resistance_knob")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    crank_joint = object_model.get_articulation("frame_to_crank")
    saddle_slide = object_model.get_articulation("frame_to_saddle_post")
    bar_slide = object_model.get_articulation("frame_to_handle_post")

    ctx.allow_overlap(
        frame,
        crank,
        elem_a="bearing_insert",
        elem_b="axle_shaft",
        reason="The crank axle is intentionally captured in the modeled bearing insert.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="bearing_boss",
        elem_b="axle_shaft",
        reason="The simplified molded bearing boss is a solid proxy around the captured crank axle.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="flywheel_cover",
        elem_b="axle_shaft",
        reason="The molded flywheel cover is simplified as a solid shell with the crank axle passing through its hidden clearance hole.",
    )
    for seam_elem in ("cover_seam_0", "cover_seam_1"):
        ctx.allow_overlap(
            frame,
            crank,
            elem_a=seam_elem,
            elem_b="axle_shaft",
            reason="The cover seam lip is simplified as a solid plastic disk with the crank axle passing through the same hidden clearance hole.",
        )
    ctx.allow_overlap(
        frame,
        saddle_knob,
        elem_a="saddle_clamp_boss",
        elem_b="screw_shank",
        reason="The clamp screw shank is intentionally threaded through the saddle clamp boss.",
    )
    ctx.allow_overlap(
        frame,
        bar_knob,
        elem_a="bar_clamp_boss",
        elem_b="screw_shank",
        reason="The clamp screw shank is intentionally threaded through the handlebar clamp boss.",
    )
    ctx.allow_overlap(
        crank,
        right_pedal,
        elem_a="right_pedal_boss",
        elem_b="spindle",
        reason="The pedal spindle is intentionally captured in the crank-arm boss.",
    )
    ctx.allow_overlap(
        crank,
        left_pedal,
        elem_a="left_pedal_boss",
        elem_b="spindle",
        reason="The pedal spindle is intentionally captured in the crank-arm boss.",
    )

    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="axle_shaft",
        outer_elem="bearing_insert",
        margin=0.001,
        name="crank axle is centered in bearing insert",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="axle_shaft",
        elem_b="bearing_insert",
        min_overlap=0.18,
        name="crank axle remains captured through bearing",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="axle_shaft",
        elem_b="bearing_boss",
        min_overlap=0.16,
        name="crank axle passes through bearing boss",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="axle_shaft",
        elem_b="flywheel_cover",
        min_overlap=0.14,
        name="crank axle passes through cover clearance",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="axle_shaft",
        elem_b="cover_seam_0",
        min_overlap=0.010,
        name="crank axle passes through side seam clearance",
    )
    ctx.expect_overlap(
        saddle_knob,
        frame,
        axes="y",
        elem_a="screw_shank",
        elem_b="saddle_clamp_boss",
        min_overlap=0.025,
        name="saddle knob screw enters clamp boss",
    )
    ctx.expect_overlap(
        bar_knob,
        frame,
        axes="y",
        elem_a="screw_shank",
        elem_b="bar_clamp_boss",
        min_overlap=0.025,
        name="handlebar knob screw enters clamp boss",
    )
    ctx.expect_contact(
        resistance_knob,
        frame,
        elem_a="dial_grip",
        elem_b="resistance_boss",
        contact_tol=0.002,
        name="resistance dial sits on molded boss",
    )
    ctx.expect_overlap(
        right_pedal,
        crank,
        axes="y",
        elem_a="spindle",
        elem_b="right_pedal_boss",
        min_overlap=0.018,
        name="right pedal spindle captured by boss",
    )
    ctx.expect_overlap(
        left_pedal,
        crank,
        axes="y",
        elem_a="spindle",
        elem_b="left_pedal_boss",
        min_overlap=0.018,
        name="left pedal spindle captured by boss",
    )
    ctx.expect_overlap(
        saddle_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="saddle_sleeve_wall_y1",
        min_overlap=0.22,
        name="saddle post retained in sleeve at low setting",
    )
    ctx.expect_overlap(
        handle_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="bar_sleeve_wall_y1",
        min_overlap=0.20,
        name="handle post retained in sleeve at low setting",
    )

    rest_saddle = ctx.part_world_position(saddle_post)
    rest_bar = ctx.part_world_position(handle_post)
    rest_arm_aabb = ctx.part_element_world_aabb(crank, elem="right_arm")
    with ctx.pose({saddle_slide: 0.160, bar_slide: 0.140, crank_joint: math.pi / 2.0}):
        ctx.expect_overlap(
            saddle_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="saddle_sleeve_wall_y1",
            min_overlap=0.060,
            name="raised saddle post still retained",
        )
        ctx.expect_overlap(
            handle_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="bar_sleeve_wall_y1",
            min_overlap=0.060,
            name="raised handle post still retained",
        )
        raised_saddle = ctx.part_world_position(saddle_post)
        raised_bar = ctx.part_world_position(handle_post)
        turned_arm_aabb = ctx.part_element_world_aabb(crank, elem="right_arm")

    ctx.check(
        "saddle adjustment moves upward",
        rest_saddle is not None and raised_saddle is not None and raised_saddle[2] > rest_saddle[2] + 0.12,
        details=f"rest={rest_saddle}, raised={raised_saddle}",
    )
    ctx.check(
        "handlebar adjustment moves upward",
        rest_bar is not None and raised_bar is not None and raised_bar[2] > rest_bar[2] + 0.10,
        details=f"rest={rest_bar}, raised={raised_bar}",
    )
    if rest_arm_aabb is not None and turned_arm_aabb is not None:
        rest_center_z = (rest_arm_aabb[0][2] + rest_arm_aabb[1][2]) * 0.5
        turned_center_z = (turned_arm_aabb[0][2] + turned_arm_aabb[1][2]) * 0.5
        ctx.check(
            "crank visibly rotates about bearing",
            turned_center_z > rest_center_z + 0.10,
            details=f"rest_z={rest_center_z}, turned_z={turned_center_z}",
        )
    else:
        ctx.fail("crank visibly rotates about bearing", "could not read crank arm AABBs")

    return ctx.report()


object_model = build_object_model()
