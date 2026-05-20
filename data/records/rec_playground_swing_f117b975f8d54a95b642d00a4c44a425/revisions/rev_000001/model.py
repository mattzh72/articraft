from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mesh,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1.0e-9:
        raise ValueError("cylinder endpoints must be separated")

    yaw = math.atan2(dy, dx)
    horizontal = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(horizontal, dz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_tube(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
) -> None:
    origin, length = _cylinder_origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_pin_y(part, name: str, center: tuple[float, float, float], *, length: float, radius: float, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, math.pi / 2.0)),
        material=material,
        name=name,
    )


def _add_rope_chord(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
) -> None:
    _add_tube(part, name, p0, p1, radius=radius, material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nest_playground_swing")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_hardware = model.material("dark_pivot_hardware", rgba=(0.05, 0.055, 0.055, 1.0))
    coated_link = model.material("black_coated_links", rgba=(0.02, 0.025, 0.025, 1.0))
    padded_blue = model.material("blue_padded_rim", rgba=(0.04, 0.24, 0.78, 1.0))
    tan_rope = model.material("tan_woven_rope", rgba=(0.83, 0.66, 0.42, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.015, 0.015, 0.014, 1.0))

    top_eye_mesh: Mesh = mesh_from_geometry(TorusGeometry(radius=0.046, tube=0.010, radial_segments=18, tubular_segments=36), "top_hanger_eye")
    lower_clip_mesh: Mesh = mesh_from_geometry(TorusGeometry(radius=0.036, tube=0.008, radial_segments=16, tubular_segments=32), "lower_basket_clip")
    basket_ring_mesh: Mesh = mesh_from_geometry(TorusGeometry(radius=0.58, tube=0.045, radial_segments=24, tubular_segments=72), "padded_basket_ring")
    inner_ring_mesh: Mesh = mesh_from_geometry(TorusGeometry(radius=0.32, tube=0.010, radial_segments=12, tubular_segments=48), "inner_rope_ring")

    frame = model.part("frame")
    # Crossbeam along the swing axis, with real A-frame side triangles at both ends.
    _add_pin_y(frame, "top_crossbeam", (0.0, 0.0, 2.25), length=1.82, radius=0.055, material=galvanized)
    for y in (-0.86, 0.86):
        frame.visual(Sphere(radius=0.078), origin=Origin(xyz=(0.0, y, 2.25)), material=galvanized, name=f"top_node_{0 if y < 0 else 1}")
        _add_tube(frame, f"front_leg_{0 if y < 0 else 1}", (0.0, y, 2.25), (0.74, y, 0.06), radius=0.040, material=galvanized)
        _add_tube(frame, f"rear_leg_{0 if y < 0 else 1}", (0.0, y, 2.25), (-0.74, y, 0.06), radius=0.040, material=galvanized)
        _add_tube(frame, f"foot_bar_{0 if y < 0 else 1}", (-0.82, y, 0.055), (0.82, y, 0.055), radius=0.030, material=galvanized)
        frame.visual(Box((1.74, 0.13, 0.030)), origin=Origin(xyz=(0.0, y, 0.015)), material=rubber, name=f"rubber_foot_{0 if y < 0 else 1}")

    _add_tube(frame, "front_ground_tie", (0.74, -0.86, 0.13), (0.74, 0.86, 0.13), radius=0.026, material=galvanized)
    _add_tube(frame, "rear_ground_tie", (-0.74, -0.86, 0.13), (-0.74, 0.86, 0.13), radius=0.026, material=galvanized)

    # Two clevis-and-pin joints under the crossbeam.  They are separate visible
    # bearings even though the two hanger branches are mimic-coupled.
    for i, y in enumerate((-0.48, 0.48)):
        frame.visual(Box((0.13, 0.018, 0.16)), origin=Origin(xyz=(0.0, y - 0.075, 2.16)), material=dark_hardware, name=f"clevis_plate_{i}_0")
        frame.visual(Box((0.13, 0.018, 0.16)), origin=Origin(xyz=(0.0, y + 0.075, 2.16)), material=dark_hardware, name=f"clevis_plate_{i}_1")
    _add_pin_y(frame, "top_pin_0", (0.0, -0.48, 2.14), length=0.15, radius=0.025, material=dark_hardware)
    _add_pin_y(frame, "top_pin_1", (0.0, 0.48, 2.14), length=0.15, radius=0.025, material=dark_hardware)

    def make_hanger(name: str, inward_y: float, rod_y: float) -> object:
        hanger = model.part(name)
        # The part origin is the top pin center.  The eye surrounds the frame pin
        # and the two rigid straps meet at this top bearing.
        hanger.visual(
            top_eye_mesh,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=coated_link,
            name="top_eye",
        )
        hanger.visual(Sphere(radius=0.025), origin=Origin(xyz=(0.0, 0.0, -0.060)), material=coated_link, name="top_boss")
        lower_center = (0.0, inward_y, -1.35)
        _add_tube(hanger, "link_0", (0.0, 0.0, -0.025), (-0.038, rod_y, -1.35), radius=0.018, material=coated_link)
        _add_tube(hanger, "link_1", (0.0, 0.0, -0.025), (0.038, rod_y, -1.35), radius=0.018, material=coated_link)
        hanger.visual(
            Cylinder(radius=0.023, length=0.16),
            origin=Origin(xyz=lower_center, rpy=(0.0, math.pi / 2.0, math.pi / 2.0)),
            material=dark_hardware,
            name="lower_pin",
        )
        return hanger

    hanger_0 = make_hanger("hanger_0", inward_y=0.05, rod_y=0.05)
    hanger_1 = make_hanger("hanger_1", inward_y=-0.05, rod_y=-0.05)

    basket = model.part("basket")
    basket.visual(basket_ring_mesh, material=padded_blue, name="padded_ring")
    basket.visual(inner_ring_mesh, origin=Origin(xyz=(0.0, 0.0, -0.025)), material=tan_rope, name="inner_rope_ring")
    # Woven chords fill the circular nest seat and terminate inside the padded rim.
    rope_r = 0.008
    chord_radius = 0.552
    for idx, y in enumerate((-0.36, -0.18, 0.0, 0.18, 0.36)):
        half = math.sqrt(chord_radius * chord_radius - y * y)
        _add_rope_chord(basket, f"weave_x_{idx}", (-half, y, -0.030), (half, y, -0.030), radius=rope_r, material=tan_rope)
    for idx, x in enumerate((-0.36, -0.18, 0.0, 0.18, 0.36)):
        half = math.sqrt(chord_radius * chord_radius - x * x)
        _add_rope_chord(basket, f"weave_y_{idx}", (x, -half, -0.036), (x, half, -0.036), radius=rope_r, material=tan_rope)
    diag_radius = 0.545
    root2 = math.sqrt(2.0)
    for idx, offset in enumerate((-0.16, 0.16)):
        # Lines parallel to x=y and x=-y give the nest a diagonally laced look.
        for family, direction in enumerate((1.0, -1.0)):
            n = (-direction / root2, 1.0 / root2)
            u = (1.0 / root2, direction / root2)
            center = (n[0] * offset, n[1] * offset)
            half = math.sqrt(diag_radius * diag_radius - offset * offset)
            p0 = (center[0] - u[0] * half, center[1] - u[1] * half, -0.044)
            p1 = (center[0] + u[0] * half, center[1] + u[1] * half, -0.044)
            _add_rope_chord(basket, f"weave_diag_{idx}_{family}", p0, p1, radius=0.0065, material=tan_rope)

    basket.visual(
        lower_clip_mesh,
        origin=Origin(xyz=(0.0, -0.50, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="lower_clip_0",
    )
    basket.visual(Box((0.18, 0.050, 0.030)), origin=Origin(xyz=(0.0, -0.50, -0.058)), material=dark_hardware, name="clip_bridge_0")
    basket.visual(
        lower_clip_mesh,
        origin=Origin(xyz=(0.0, 0.50, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="lower_clip_1",
    )
    basket.visual(Box((0.18, 0.050, 0.030)), origin=Origin(xyz=(0.0, 0.50, -0.058)), material=dark_hardware, name="clip_bridge_1")
    for y, prefix in ((-0.50, "front"), (0.50, "rear")):
        _add_tube(basket, f"{prefix}_clip_strap_0", (-0.08, y, -0.058), (-0.30, y, -0.030), radius=0.010, material=dark_hardware)
        _add_tube(basket, f"{prefix}_clip_strap_1", (0.08, y, -0.058), (0.30, y, -0.030), radius=0.010, material=dark_hardware)

    top_0 = model.articulation(
        "top_pivot_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hanger_0,
        origin=Origin(xyz=(0.0, -0.48, 2.14)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "top_pivot_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hanger_1,
        origin=Origin(xyz=(0.0, 0.48, 2.14)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=-0.55, upper=0.55),
        mimic=Mimic(joint="top_pivot_0", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=hanger_0,
        child=basket,
        # The child frame is the center of the nest seat.  The Y-axis passes
        # through both lower basket clips, so the seat remains captured while it
        # rocks slightly relative to the rigid hanger links.
        origin=Origin(xyz=(0.0, 0.48, -1.35)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.18, upper=0.18),
    )
    model.meta["primary_top_joint"] = top_0.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    hanger_0 = object_model.get_part("hanger_0")
    hanger_1 = object_model.get_part("hanger_1")
    basket = object_model.get_part("basket")
    top_0 = object_model.get_articulation("top_pivot_0")
    top_1 = object_model.get_articulation("top_pivot_1")
    lower = object_model.get_articulation("lower_pivot")

    ctx.check(
        "two visible top pivots are revolute",
        top_0.articulation_type == ArticulationType.REVOLUTE
        and top_1.articulation_type == ArticulationType.REVOLUTE
        and top_1.mimic is not None
        and top_1.mimic.joint == "top_pivot_0",
        details=f"top_0={top_0.articulation_type}, top_1={top_1.articulation_type}, mimic={top_1.mimic}",
    )
    ctx.check(
        "lower basket pivot has small rocking range",
        lower.articulation_type == ArticulationType.REVOLUTE
        and lower.motion_limits is not None
        and lower.motion_limits.lower is not None
        and lower.motion_limits.upper is not None
        and lower.motion_limits.lower < 0.0 < lower.motion_limits.upper
        and lower.motion_limits.upper <= 0.20,
        details=f"type={lower.articulation_type}, limits={lower.motion_limits}",
    )

    # At rest both top eyes are concentric with the frame pins, and the lower
    # basket clips surround the two lower hanger pins.
    ctx.expect_overlap(frame, hanger_0, axes="xz", elem_a="top_pin_0", elem_b="top_eye", min_overlap=0.035, name="hanger 0 is captured by top pin")
    ctx.expect_overlap(frame, hanger_1, axes="xz", elem_a="top_pin_1", elem_b="top_eye", min_overlap=0.035, name="hanger 1 is captured by top pin")
    ctx.expect_overlap(hanger_0, basket, axes="xyz", elem_a="lower_pin", elem_b="lower_clip_0", min_overlap=0.012, name="basket clip stays around lower pin 0")
    ctx.expect_overlap(hanger_1, basket, axes="xyz", elem_a="lower_pin", elem_b="lower_clip_1", min_overlap=0.012, name="basket clip stays around lower pin 1")

    rest_pos = ctx.part_world_position(basket)
    rest_aabb = ctx.part_world_aabb(basket)
    rest_height = None if rest_aabb is None else rest_aabb[1][2] - rest_aabb[0][2]

    with ctx.pose({top_0: 0.35}):
        swung_pos = ctx.part_world_position(basket)
        ctx.expect_overlap(hanger_0, basket, axes="xyz", elem_a="lower_pin", elem_b="lower_clip_0", min_overlap=0.012, name="basket remains clipped to pin 0 while swinging")
        ctx.expect_overlap(hanger_1, basket, axes="xyz", elem_a="lower_pin", elem_b="lower_clip_1", min_overlap=0.012, name="basket remains clipped to pin 1 while swinging")

    ctx.check(
        "top pivot swings the nest assembly as a unit",
        rest_pos is not None
        and swung_pos is not None
        and abs(swung_pos[0] - rest_pos[0]) > 0.25
        and swung_pos[2] > rest_pos[2] + 0.04,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    with ctx.pose({lower: 0.16}):
        tilted_aabb = ctx.part_world_aabb(basket)
        tilted_height = None if tilted_aabb is None else tilted_aabb[1][2] - tilted_aabb[0][2]
        ctx.expect_overlap(hanger_0, basket, axes="xyz", elem_a="lower_pin", elem_b="lower_clip_0", min_overlap=0.012, name="tilted seat remains clipped to lower pin 0")
        ctx.expect_overlap(hanger_1, basket, axes="xyz", elem_a="lower_pin", elem_b="lower_clip_1", min_overlap=0.012, name="tilted seat remains clipped to lower pin 1")

    ctx.check(
        "lower pivot visibly rocks the basket frame",
        rest_height is not None and tilted_height is not None and tilted_height > rest_height + 0.06,
        details=f"rest_height={rest_height}, tilted_height={tilted_height}",
    )

    return ctx.report()


object_model = build_object_model()
