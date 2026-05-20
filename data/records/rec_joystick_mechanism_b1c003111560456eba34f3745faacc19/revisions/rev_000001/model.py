from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (-math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _box(part, name: str, size, xyz, material: str, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, axis: str, material: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def _torus(part, name: str, major: float, tube: float, xyz, axis: str, material: str) -> None:
    part.visual(
        mesh_from_geometry(
            TorusGeometry(radius=major, tube=tube, radial_segments=20, tubular_segments=48),
            name,
        ),
        origin=Origin(xyz=xyz, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def _helix_between(start, end, *, coils: int, coil_radius: float, wire_radius: float, name: str):
    sx, sy, sz = start
    ex, ey, ez = end
    axis = (ex - sx, ey - sy, ez - sz)
    length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    ux, uy, uz = (axis[0] / length, axis[1] / length, axis[2] / length)

    # Pick a stable orthonormal pair around the spring axis.
    ref = (0.0, 0.0, 1.0) if abs(uz) < 0.90 else (1.0, 0.0, 0.0)
    vx = uy * ref[2] - uz * ref[1]
    vy = uz * ref[0] - ux * ref[2]
    vz = ux * ref[1] - uy * ref[0]
    v_len = math.sqrt(vx * vx + vy * vy + vz * vz)
    vx, vy, vz = vx / v_len, vy / v_len, vz / v_len
    wx = uy * vz - uz * vy
    wy = uz * vx - ux * vz
    wz = ux * vy - uy * vx

    pts = []
    steps = coils * 18
    for i in range(steps + 1):
        t = i / steps
        theta = 2.0 * math.pi * coils * t
        cx = sx + axis[0] * t
        cy = sy + axis[1] * t
        cz = sz + axis[2] * t
        pts.append(
            (
                cx + coil_radius * (math.cos(theta) * vx + math.sin(theta) * wx),
                cy + coil_radius * (math.cos(theta) * vy + math.sin(theta) * wy),
                cz + coil_radius * (math.cos(theta) * vz + math.sin(theta) * wz),
            )
        )
    return mesh_from_geometry(
        wire_from_points(pts, radius=wire_radius, radial_segments=8, cap_ends=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gimballed_joystick_mechanism")

    model.material("black_oxide", rgba=(0.015, 0.016, 0.017, 1.0))
    model.material("machined_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    model.material("dark_hardcoat", rgba=(0.075, 0.080, 0.085, 1.0))
    model.material("bronze_bushing", rgba=(0.67, 0.42, 0.16, 1.0))
    model.material("zinc_fastener", rgba=(0.72, 0.74, 0.70, 1.0))
    model.material("spring_steel", rgba=(0.50, 0.55, 0.58, 1.0))
    model.material("matte_rubber", rgba=(0.012, 0.012, 0.011, 1.0))

    pivot_z = 0.145

    base = model.part("base_frame")
    _box(base, "ground_plate", (0.360, 0.360, 0.018), (0.0, 0.0, 0.009), "black_oxide")
    _box(base, "machined_top_plate", (0.290, 0.290, 0.010), (0.0, 0.0, 0.023), "dark_hardcoat")

    # Squared, fabricated bearing towers.  The center is deliberately open so the
    # roll trunnions pass through real clearance instead of intersecting a block.
    for side, x in (("neg", -0.158), ("pos", 0.158)):
        _box(base, f"{side}_tower_foot", (0.052, 0.240, 0.020), (x, 0.0, 0.036), "black_oxide")
        for y in (-0.060, 0.060):
            _box(base, f"{side}_tower_post_{'front' if y < 0 else 'rear'}", (0.026, 0.026, 0.174), (x, y, 0.112), "black_oxide")
        _box(base, f"{side}_tower_cap", (0.026, 0.146, 0.018), (x, 0.0, 0.205), "black_oxide")
        _box(base, f"{side}_tower_lower_tie", (0.026, 0.128, 0.014), (x, 0.0, pivot_z - 0.047), "black_oxide")
        _box(base, f"{side}_bearing_yoke_front", (0.028, 0.028, 0.010), (x, -0.038, pivot_z), "black_oxide")
        _box(base, f"{side}_bearing_yoke_rear", (0.028, 0.028, 0.010), (x, 0.038, pivot_z), "black_oxide")
        _box(base, f"{side}_bearing_yoke_top", (0.028, 0.010, 0.026), (x, 0.0, pivot_z + 0.039), "black_oxide")
        _box(base, f"{side}_bearing_yoke_bottom", (0.028, 0.010, 0.024), (x, 0.0, pivot_z - 0.038), "black_oxide")
        _torus(base, f"{side}_roll_bearing", 0.018, 0.0060, (x, 0.0, pivot_z), "x", "bronze_bushing")
        _torus(base, f"{side}_retainer_cap", 0.018, 0.0030, (x + (0.008 if x > 0 else -0.008), 0.0, pivot_z), "x", "zinc_fastener")

    # Removable-cover rails and screw bosses on the open front and rear faces.
    for side, y in (("front", -0.173), ("rear", 0.173)):
        _box(base, f"{side}_cover_mount", (0.184, 0.012, 0.076), (0.0, y, 0.056), "black_oxide")
        _box(base, f"{side}_cover_upper_guide", (0.150, 0.018, 0.008), (0.0, y, 0.098), "machined_steel")
        _box(base, f"{side}_cover_lower_guide", (0.150, 0.018, 0.008), (0.0, y, 0.018), "machined_steel")

    # Boot-retainer hardware: an exposed lower ring held on four stand-offs.
    _torus(base, "lower_boot_ring", 0.059, 0.0045, (0.0, 0.0, 0.065), "z", "matte_rubber")
    for i, (x, y) in enumerate(((0.059, 0.0), (-0.059, 0.0), (0.0, 0.059), (0.0, -0.059))):
        _cyl(base, f"boot_standoff_{i}", 0.004, 0.044, (x, y, 0.044), "z", "machined_steel")
        _cyl(base, f"boot_screw_{i}", 0.006, 0.004, (x, y, 0.069), "z", "zinc_fastener")

    # Spring anchor blocks and neutral return springs.  The springs are fixed to
    # the study base; their top eyes line up with the centering plate at neutral.
    spring_pairs = [
        ((0.136, 0.136, 0.048), (0.098, 0.098, 0.088)),
        ((-0.136, 0.136, 0.048), (-0.098, 0.098, 0.088)),
        ((-0.136, -0.136, 0.048), (-0.098, -0.098, 0.088)),
        ((0.136, -0.136, 0.048), (0.098, -0.098, 0.088)),
    ]
    for i, (lower, upper) in enumerate(spring_pairs):
        _box(base, f"spring_pedestal_{i}", (0.018, 0.018, 0.013), (lower[0], lower[1], 0.0345), "black_oxide")
        _box(base, f"spring_anchor_{i}", (0.026, 0.026, 0.014), lower, "black_oxide")
        _cyl(base, f"spring_eye_{i}", 0.006, 0.020, lower, "x" if i % 2 == 0 else "y", "zinc_fastener")
        base.visual(
            _helix_between(lower, upper, coils=7, coil_radius=0.006, wire_radius=0.0011, name=f"return_spring_{i}"),
            material="spring_steel",
            name=f"return_spring_{i}",
        )

    # Visible socket-head fasteners tie the feet and the cover rails to the base.
    for i, (x, y) in enumerate(((-0.145, -0.145), (-0.145, 0.145), (0.145, -0.145), (0.145, 0.145))):
        _cyl(base, f"base_cap_screw_{i}", 0.007, 0.005, (x, y, 0.0305), "z", "zinc_fastener")
        _cyl(base, f"leveling_foot_{i}", 0.014, 0.010, (x, y, -0.005), "z", "black_oxide")

    roll = model.part("roll_yoke")
    # A nested outer cage, with clearance windows around the pitch axis and
    # short roll-axis trunnions captured by the base tower bearings.
    for side, y in (("front", -0.075), ("rear", 0.075)):
        _box(roll, f"{side}_pitch_frame_top", (0.176, 0.014, 0.012), (0.0, y, 0.042), "dark_hardcoat")
        _box(roll, f"{side}_pitch_frame_bottom", (0.176, 0.014, 0.012), (0.0, y, -0.042), "dark_hardcoat")
        _box(roll, f"{side}_pitch_frame_left", (0.014, 0.014, 0.084), (-0.081, y, 0.0), "dark_hardcoat")
        _box(roll, f"{side}_pitch_frame_right", (0.014, 0.014, 0.084), (0.081, y, 0.0), "dark_hardcoat")
        _box(roll, f"{side}_bearing_top_bridge", (0.058, 0.014, 0.012), (0.0, y, 0.030), "dark_hardcoat")
        _box(roll, f"{side}_bearing_bottom_bridge", (0.058, 0.014, 0.012), (0.0, y, -0.030), "dark_hardcoat")
        _torus(roll, f"{side}_pitch_bearing", 0.020, 0.0045, (0.0, y, 0.0), "y", "bronze_bushing")

    for x in (-0.088, 0.088):
        _box(roll, f"roll_side_beam_{'neg' if x < 0 else 'pos'}", (0.018, 0.164, 0.018), (x, 0.0, 0.0), "dark_hardcoat")
        _box(roll, f"roll_upper_tie_{'neg' if x < 0 else 'pos'}", (0.014, 0.164, 0.012), (x, 0.0, 0.042), "dark_hardcoat")
        _box(roll, f"roll_lower_tie_{'neg' if x < 0 else 'pos'}", (0.014, 0.164, 0.012), (x, 0.0, -0.042), "dark_hardcoat")
        _cyl(roll, f"roll_trunnion_{'neg' if x < 0 else 'pos'}", 0.012, 0.100, (math.copysign(0.130, x), 0.0, 0.0), "x", "machined_steel")
        _cyl(roll, f"roll_end_washer_{'neg' if x < 0 else 'pos'}", 0.019, 0.006, (math.copysign(0.183, x), 0.0, 0.0), "x", "zinc_fastener")

    # Small anchor ears on the cage make the return-spring load path legible.
    for i, (x, y) in enumerate(((0.054, 0.054), (-0.054, 0.054), (-0.054, -0.054), (0.054, -0.054))):
        _box(roll, f"moving_anchor_strut_{i}", (0.034, 0.010, 0.010), (math.copysign(0.071, x), y, -0.047), "machined_steel")
        _box(roll, f"moving_anchor_lug_{i}", (0.016, 0.010, 0.014), (x, y, -0.047), "machined_steel")
        _cyl(roll, f"moving_anchor_hole_{i}", 0.004, 0.018, (x, y, -0.047), "x" if abs(y) > abs(x) else "y", "zinc_fastener")

    pitch = model.part("pitch_yoke")
    _cyl(pitch, "pitch_trunnion", 0.016, 0.194, (0.0, 0.0, 0.0), "y", "machined_steel")
    _cyl(pitch, "central_cross_boss", 0.026, 0.038, (0.0, 0.0, 0.0), "z", "machined_steel")
    _box(pitch, "stick_saddle", (0.058, 0.040, 0.026), (0.0, 0.0, 0.025), "dark_hardcoat")
    _box(pitch, "split_clamp_left", (0.014, 0.052, 0.034), (-0.023, 0.0, 0.037), "black_oxide")
    _box(pitch, "split_clamp_right", (0.014, 0.052, 0.034), (0.023, 0.0, 0.037), "black_oxide")
    for x in (-0.030, 0.030):
        _cyl(pitch, f"clamp_bolt_{'neg' if x < 0 else 'pos'}", 0.004, 0.060, (x, 0.0, 0.043), "y", "zinc_fastener")
    for y in (-0.100, 0.100):
        _cyl(pitch, f"pitch_end_washer_{'front' if y < 0 else 'rear'}", 0.019, 0.006, (0.0, y, 0.0), "y", "zinc_fastener")

    stick = model.part("control_stick")
    _cyl(stick, "lower_plug", 0.012, 0.022, (0.0, 0.0, 0.049), "z", "machined_steel")
    _cyl(stick, "main_shaft", 0.011, 0.300, (0.0, 0.0, 0.205), "z", "machined_steel")
    _cyl(stick, "machined_sleeve", 0.017, 0.066, (0.0, 0.0, 0.112), "z", "black_oxide")
    _torus(stick, "upper_boot_ring", 0.036, 0.004, (0.0, 0.0, 0.105), "z", "matte_rubber")
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        x = 0.022 * math.cos(angle)
        y = 0.022 * math.sin(angle)
        _box(stick, f"boot_ring_spoke_{i}", (0.026 if abs(x) > abs(y) else 0.006, 0.006 if abs(x) > abs(y) else 0.026, 0.005), (x / 2.0, y / 2.0, 0.105), "matte_rubber")
    _cyl(stick, "plain_top_cap", 0.015, 0.018, (0.0, 0.0, 0.362), "z", "black_oxide")
    _cyl(stick, "centering_disc", 0.045, 0.006, (0.0, 0.0, 0.110), "z", "machined_steel")

    for i, (x, y) in enumerate(((0.052, 0.052), (-0.052, 0.052), (-0.052, -0.052), (0.052, -0.052))):
        _box(
            stick,
            f"centering_tab_{i}",
            (0.046, 0.008, 0.006),
            (0.036 * (1.0 if x > 0 else -1.0), 0.036 * (1.0 if y > 0 else -1.0), 0.110),
            "machined_steel",
            rpy=(0.0, 0.0, math.atan2(y, x)),
        )
        _cyl(stick, f"centering_eye_{i}", 0.0045, 0.020, (x, y, 0.110), "x" if i % 2 == 0 else "y", "zinc_fastener")

    front_cover = model.part("front_cover")
    _box(front_cover, "cover_plate", (0.152, 0.006, 0.072), (0.0, 0.0, 0.0), "dark_hardcoat")
    _box(front_cover, "raised_access_rib", (0.118, 0.003, 0.010), (0.0, -0.0045, 0.020), "machined_steel")
    for i, (x, z) in enumerate(((-0.058, -0.022), (0.058, -0.022), (-0.058, 0.022), (0.058, 0.022))):
        _cyl(front_cover, f"cover_screw_{i}", 0.006, 0.005, (x, -0.004, z), "y", "zinc_fastener")

    rear_cover = model.part("rear_cover")
    _box(rear_cover, "cover_plate", (0.152, 0.006, 0.072), (0.0, 0.0, 0.0), "dark_hardcoat")
    _box(rear_cover, "raised_access_rib", (0.118, 0.003, 0.010), (0.0, 0.0045, 0.020), "machined_steel")
    for i, (x, z) in enumerate(((-0.058, -0.022), (0.058, -0.022), (-0.058, 0.022), (0.058, 0.022))):
        _cyl(rear_cover, f"cover_screw_{i}", 0.006, 0.005, (x, 0.004, z), "y", "zinc_fastener")

    model.articulation(
        "base_to_roll",
        ArticulationType.REVOLUTE,
        parent=base,
        child=roll,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.26, upper=0.26),
        motion_properties=MotionProperties(damping=0.30, friction=0.08),
    )
    model.articulation(
        "roll_to_pitch",
        ArticulationType.REVOLUTE,
        parent=roll,
        child=pitch,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=-0.24, upper=0.24),
        motion_properties=MotionProperties(damping=0.30, friction=0.08),
    )
    model.articulation(
        "pitch_to_stick",
        ArticulationType.FIXED,
        parent=pitch,
        child=stick,
        origin=Origin(),
    )
    model.articulation(
        "front_cover_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=front_cover,
        origin=Origin(xyz=(0.0, -0.182, 0.056)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.20, lower=0.0, upper=0.052),
    )
    model.articulation(
        "rear_cover_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=rear_cover,
        origin=Origin(xyz=(0.0, 0.182, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.20, lower=0.0, upper=0.052),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    roll = object_model.get_part("roll_yoke")
    pitch = object_model.get_part("pitch_yoke")
    stick = object_model.get_part("control_stick")
    front_cover = object_model.get_part("front_cover")
    rear_cover = object_model.get_part("rear_cover")
    roll_joint = object_model.get_articulation("base_to_roll")
    pitch_joint = object_model.get_articulation("roll_to_pitch")
    front_slide = object_model.get_articulation("front_cover_slide")
    rear_slide = object_model.get_articulation("rear_cover_slide")

    for side in ("neg", "pos"):
        ctx.allow_overlap(
            base,
            roll,
            elem_a=f"{side}_roll_bearing",
            elem_b=f"roll_trunnion_{side}",
            reason="Roll-axis trunnion is intentionally represented as a lightly seated shaft inside the exposed bearing ring.",
        )
    for side in ("front", "rear"):
        ctx.allow_overlap(
            roll,
            pitch,
            elem_a=f"{side}_pitch_bearing",
            elem_b="pitch_trunnion",
            reason="Pitch-axis trunnion is intentionally captured by the nested yoke bearing ring with a tiny seated fit.",
        )

    ctx.expect_overlap(
        roll,
        base,
        axes="x",
        elem_a="roll_trunnion_neg",
        elem_b="neg_roll_bearing",
        min_overlap=0.010,
        name="negative roll trunnion is captured in bearing",
    )
    ctx.expect_overlap(
        roll,
        base,
        axes="x",
        elem_a="roll_trunnion_pos",
        elem_b="pos_roll_bearing",
        min_overlap=0.010,
        name="positive roll trunnion is captured in bearing",
    )
    ctx.expect_overlap(
        pitch,
        roll,
        axes="y",
        elem_a="pitch_trunnion",
        elem_b="front_pitch_bearing",
        min_overlap=0.006,
        name="pitch trunnion spans front bearing",
    )
    ctx.expect_overlap(
        stick,
        base,
        axes="xy",
        elem_a="main_shaft",
        elem_b="lower_boot_ring",
        min_overlap=0.010,
        name="shaft passes through lower boot retainer footprint",
    )
    ctx.expect_gap(
        stick,
        pitch,
        axis="z",
        positive_elem="lower_plug",
        negative_elem="stick_saddle",
        max_gap=0.003,
        max_penetration=0.0,
        name="stick plug seats on pitch saddle without overlap",
    )
    ctx.expect_contact(
        front_cover,
        base,
        elem_a="cover_plate",
        elem_b="front_cover_mount",
        contact_tol=0.004,
        name="front access cover is seated on its guide rail",
    )
    ctx.expect_contact(
        rear_cover,
        base,
        elem_a="cover_plate",
        elem_b="rear_cover_mount",
        contact_tol=0.004,
        name="rear access cover is seated on its guide rail",
    )

    rest_stick = ctx.part_element_world_aabb(stick, elem="plain_top_cap")
    with ctx.pose({pitch_joint: 0.20}):
        pitched_stick = ctx.part_element_world_aabb(stick, elem="plain_top_cap")
    ctx.check(
        "pitch joint moves stick top in x",
        rest_stick is not None
        and pitched_stick is not None
        and pitched_stick[1][0] > rest_stick[1][0] + 0.035,
        details=f"rest={rest_stick}, pitched={pitched_stick}",
    )

    with ctx.pose({roll_joint: 0.20}):
        rolled_stick = ctx.part_element_world_aabb(stick, elem="plain_top_cap")
    ctx.check(
        "roll joint moves stick top in y",
        rest_stick is not None
        and rolled_stick is not None
        and rolled_stick[0][1] < rest_stick[0][1] - 0.035,
        details=f"rest={rest_stick}, rolled={rolled_stick}",
    )

    front_rest = ctx.part_world_position(front_cover)
    rear_rest = ctx.part_world_position(rear_cover)
    with ctx.pose({front_slide: 0.040, rear_slide: 0.040}):
        front_removed = ctx.part_world_position(front_cover)
        rear_removed = ctx.part_world_position(rear_cover)
    ctx.check(
        "covers slide outward for removal",
        front_rest is not None
        and rear_rest is not None
        and front_removed is not None
        and rear_removed is not None
        and front_removed[1] < front_rest[1] - 0.035
        and rear_removed[1] > rear_rest[1] + 0.035,
        details=f"front {front_rest}->{front_removed}, rear {rear_rest}->{rear_removed}",
    )

    return ctx.report()


object_model = build_object_model()
