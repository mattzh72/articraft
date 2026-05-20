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
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(tuple(size)), origin=Origin(xyz=tuple(xyz), rpy=tuple(rpy)), material=material, name=name)


def _cylinder(part, name: str, radius: float, length: float, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=tuple(xyz), rpy=tuple(rpy)), material=material, name=name)


def _lip_nose_mesh(width: float = 1.90) -> MeshGeometry:
    """A tapered beveled truck-plate nose, authored in the lip frame."""
    x0, x1 = 0.70, 0.92
    half_w = width / 2.0
    z_top = 0.0
    z_rear_bottom = -0.070
    z_front_bottom = -0.025
    vertices = [
        (x0, -half_w, z_top),
        (x1, -half_w, z_top),
        (x1, half_w, z_top),
        (x0, half_w, z_top),
        (x0, -half_w, z_rear_bottom),
        (x1, -half_w, z_front_bottom),
        (x1, half_w, z_front_bottom),
        (x0, half_w, z_rear_bottom),
    ]
    faces = [
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1),
        (3, 2, 6), (3, 6, 7),
        (0, 3, 7), (0, 7, 4),
        (1, 5, 6), (1, 6, 2),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_dock_loading_ramp")

    concrete = model.material("aged_concrete", rgba=(0.48, 0.49, 0.47, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.03, 0.035, 0.035, 1.0))
    deck_steel = model.material("painted_safety_blue_steel", rgba=(0.05, 0.17, 0.28, 1.0))
    worn_steel = model.material("worn_galvanized_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    yellow = model.material("safety_yellow_paint", rgba=(0.95, 0.72, 0.07, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    hydraulic_blue = model.material("hydraulic_cylinder_blue", rgba=(0.02, 0.08, 0.18, 1.0))
    chrome = model.material("polished_hydraulic_rod", rgba=(0.78, 0.80, 0.78, 1.0))

    frame = model.part("dock_frame")

    # Concrete dock, pit, and front steel curb.  The root frame origin is the
    # rear hinge line of the leveler at dock-floor height.
    _box(frame, "concrete_dock", (1.00, 2.90, 0.90), (-0.62, 0.0, -0.45), concrete)
    _box(frame, "pit_floor", (2.90, 2.78, 0.12), (1.32, 0.0, -0.84), concrete)
    _box(frame, "side_wall_0", (2.85, 0.16, 0.82), (1.32, -1.36, -0.44), concrete)
    _box(frame, "side_wall_1", (2.85, 0.16, 0.82), (1.32, 1.36, -0.44), concrete)
    _box(frame, "rear_curb", (0.10, 2.76, 0.16), (-0.15, 0.0, -0.07), dark_steel)
    _box(frame, "yellow_dock_edge", (0.11, 2.62, 0.018), (-0.11, 0.0, 0.010), yellow)
    _box(frame, "pit_crossmember", (2.72, 0.12, 0.16), (1.30, 0.0, -0.54), dark_steel)
    _box(frame, "front_crossmember", (0.16, 2.72, 0.22), (2.72, 0.0, -0.55), dark_steel)

    # Rubber bumpers and bolted plates mounted to the dock face.
    for idx, y in enumerate((-1.17, 1.17)):
        _box(frame, f"rubber_bumper_{idx}", (0.16, 0.26, 0.58), (0.035, y, -0.35), rubber)
        _box(frame, f"bumper_backplate_{idx}", (0.045, 0.36, 0.68), (-0.055, y, -0.35), dark_steel)
        for j, z in enumerate((-0.57, -0.35, -0.13)):
            _cylinder(frame, f"bumper_bolt_{idx}_{j}", 0.030, 0.020, (0.07, y, z), worn_steel, rpy=(0.0, math.pi / 2.0, 0.0))

    # Massive rear hinge knuckles fixed to the dock frame.
    for idx, y in enumerate((-0.92, 0.0, 0.92)):
        _cylinder(frame, f"fixed_hinge_knuckle_{idx}", 0.076, 0.34, (0.0, y, -0.060), dark_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
    _cylinder(frame, "rear_hinge_pin", 0.030, 2.38, (0.0, 0.0, -0.060), worn_steel, rpy=(math.pi / 2.0, 0.0, 0.0))

    # Side guard rails around the pit so the assembly reads as warehouse equipment.
    for side_idx, y in enumerate((-1.47, 1.47)):
        for post_idx, x in enumerate((0.35, 1.30, 2.30)):
            _box(frame, f"guard_post_{side_idx}_{post_idx}", (0.09, 0.09, 0.70), (x, y, 0.27), yellow)
        _box(frame, f"guard_rail_{side_idx}", (2.16, 0.07, 0.08), (1.325, y, 0.60), yellow)
        _box(frame, f"lower_guard_rail_{side_idx}", (2.16, 0.055, 0.055), (1.325, y, 0.31), yellow)

    # Heavy but simplified base-mounted lift cylinders under the deck.  They are
    # connected to the pit floor with clevis plates and visually meet the ramp lugs.
    ram_angle = -0.42
    for idx, y in enumerate((-0.72, 0.72)):
        _box(frame, f"ram_base_plate_{idx}", (0.34, 0.24, 0.06), (0.46, y, -0.755), dark_steel)
        _box(frame, f"ram_clevis_0_{idx}", (0.09, 0.035, 0.26), (0.36, y - 0.075, -0.64), dark_steel)
        _box(frame, f"ram_clevis_1_{idx}", (0.09, 0.035, 0.26), (0.36, y + 0.075, -0.64), dark_steel)
        _box(frame, f"ram_body_{idx}", (0.86, 0.105, 0.105), (0.82, y, -0.52), hydraulic_blue, rpy=(0.0, ram_angle, 0.0))
        _box(frame, f"ram_rod_collar_{idx}", (0.26, 0.13, 0.18), (1.18, y, -0.50), hydraulic_blue)
        _box(frame, f"ram_rod_{idx}", (0.86, 0.055, 0.055), (1.42, y, -0.50), chrome, rpy=(0.0, ram_angle, 0.0))
        _cylinder(frame, f"ram_pivot_pin_{idx}", 0.038, 0.24, (0.36, y, -0.64), worn_steel, rpy=(math.pi / 2.0, 0.0, 0.0))

    deck = model.part("ramp_deck")
    # The deck frame is at the rear hinge line.  The load-bearing plate starts a
    # little forward of the knuckles, with hinge leaves bridging the gap.
    _box(deck, "main_deck_plate", (2.34, 2.08, 0.090), (1.29, 0.0, -0.050), deck_steel)
    _box(deck, "side_curb_0", (2.30, 0.14, 0.22), (1.42, -1.10, -0.040), dark_steel)
    _box(deck, "side_curb_1", (2.30, 0.14, 0.22), (1.42, 1.10, -0.040), dark_steel)
    for idx, x in enumerate((0.40, 0.95, 2.05, 2.48)):
        _box(deck, f"underside_rib_{idx}", (0.070, 1.84, 0.18), (x, 0.0, -0.165), dark_steel)
    # The mid rib is split around the hydraulic rod paths, like a real rib with
    # torch-cut service windows.
    for name, y, span in (("lower", -0.88, 0.18), ("middle", 0.0, 1.10), ("upper", 0.88, 0.18)):
        _box(deck, f"underside_rib_window_{name}", (0.070, span, 0.18), (1.50, y, -0.165), dark_steel)
    for idx, y in enumerate((-0.48, 0.48)):
        _cylinder(deck, f"moving_hinge_knuckle_{idx}", 0.070, 0.30, (0.0, y, -0.060), dark_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
        _box(deck, f"rear_hinge_leaf_{idx}", (0.28, 0.24, 0.045), (0.180, y, -0.070), dark_steel)

    # Raised diamond-like tread bars and bolt heads, embedded slightly into the
    # top plate so they are part of the same welded deck assembly.
    tread_id = 0
    for x in (0.42, 0.82, 1.22, 1.62, 2.02, 2.42):
        for y in (-0.66, 0.0, 0.66):
            yaw = math.radians(35.0 if (tread_id % 2 == 0) else -35.0)
            _box(deck, f"raised_tread_{tread_id}", (0.34, 0.032, 0.018), (x, y, 0.002), worn_steel, rpy=(0.0, 0.0, yaw))
            tread_id += 1
    for i, x in enumerate((0.32, 1.20, 2.08, 2.32)):
        for j, y in enumerate((-0.92, 0.92)):
            _cylinder(deck, f"deck_edge_bolt_{i}_{j}", 0.028, 0.018, (x, y, 0.003), worn_steel)

    # Lugs on the underside line up with the fixed hydraulic rods at the closed pose.
    for idx, y in enumerate((-0.72, 0.72)):
        _box(deck, f"ram_lug_web_{idx}_0", (0.10, 0.040, 0.25), (1.78, y - 0.080, -0.210), dark_steel)
        _box(deck, f"ram_lug_web_{idx}_1", (0.10, 0.040, 0.25), (1.78, y + 0.080, -0.210), dark_steel)
        _box(deck, f"ram_top_lug_{idx}", (0.18, 0.13, 0.16), (1.78, y, -0.340), dark_steel)
        _cylinder(deck, f"ram_top_pin_{idx}", 0.032, 0.18, (1.78, y, -0.340), worn_steel, rpy=(math.pi / 2.0, 0.0, 0.0))

    # Forward lip hinge sockets fixed to the deck.
    for idx, y in enumerate((-0.80, 0.0, 0.80)):
        _box(deck, f"front_hinge_leaf_{idx}", (0.28, 0.22, 0.050), (2.50, y, -0.055), dark_steel)
        _cylinder(deck, f"front_hinge_knuckle_{idx}", 0.055, 0.30, (2.62, y, -0.040), dark_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
    _cylinder(deck, "front_hinge_pin", 0.024, 2.00, (2.62, 0.0, -0.040), worn_steel, rpy=(math.pi / 2.0, 0.0, 0.0))

    lip = model.part("lip_plate")
    _box(lip, "lip_back_plate", (0.64, 1.90, 0.070), (0.42, 0.0, -0.035), worn_steel)
    lip.visual(mesh_from_geometry(_lip_nose_mesh(), "tapered_lip_nose"), origin=Origin(), material=worn_steel, name="tapered_lip_nose")
    for idx, y in enumerate((-0.45, 0.45)):
        _cylinder(lip, f"lip_hinge_knuckle_{idx}", 0.050, 0.28, (0.0, y, -0.040), dark_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
        _box(lip, f"lip_hinge_leaf_{idx}", (0.25, 0.24, 0.050), (0.165, y, -0.043), dark_steel)
    for i, x in enumerate((0.30, 0.56, 0.82)):
        _box(lip, f"lip_traction_bar_{i}", (0.030, 1.58, 0.018), (x, 0.0, 0.002), dark_steel)

    deck_hinge = model.articulation(
        "frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=0.25, lower=-0.18, upper=0.34),
        motion_properties=MotionProperties(damping=250.0, friction=60.0),
    )
    deck_hinge.meta["description"] = "Rear dock-leveler hinge; positive motion raises the free end of the deck."

    lip_hinge = model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(2.62, 0.0, -0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.45, lower=0.0, upper=0.55),
        motion_properties=MotionProperties(damping=80.0, friction=25.0),
    )
    lip_hinge.meta["description"] = "Forward hinged lip that rotates downward to bridge to a truck bed."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("dock_frame")
    deck = object_model.get_part("ramp_deck")
    lip = object_model.get_part("lip_plate")
    deck_hinge = object_model.get_articulation("frame_to_deck")
    lip_hinge = object_model.get_articulation("deck_to_lip")

    ctx.allow_overlap(
        frame,
        deck,
        elem_a="rear_hinge_pin",
        elem_b="moving_hinge_knuckle_0",
        reason="The hinge pin is intentionally captured inside the moving deck knuckle.",
    )
    ctx.allow_overlap(
        frame,
        deck,
        elem_a="rear_hinge_pin",
        elem_b="moving_hinge_knuckle_1",
        reason="The hinge pin is intentionally captured inside the moving deck knuckle.",
    )
    ctx.allow_overlap(
        lip,
        deck,
        elem_a="lip_hinge_knuckle_0",
        elem_b="front_hinge_pin",
        reason="The front hinge pin is intentionally captured inside the hinged lip knuckle.",
    )
    ctx.allow_overlap(
        lip,
        deck,
        elem_a="lip_hinge_knuckle_1",
        elem_b="front_hinge_pin",
        reason="The front hinge pin is intentionally captured inside the hinged lip knuckle.",
    )
    for idx in (0, 1):
        ctx.allow_overlap(
            frame,
            deck,
            elem_a=f"ram_rod_{idx}",
            elem_b=f"ram_top_lug_{idx}",
            reason="The hydraulic rod end is intentionally seated into the deck lift lug clevis.",
        )
        ctx.allow_overlap(
            frame,
            deck,
            elem_a=f"ram_rod_{idx}",
            elem_b=f"ram_top_pin_{idx}",
            reason="The rod eye is intentionally captured around the deck lift-lug pin.",
        )

    with ctx.pose({deck_hinge: 0.0, lip_hinge: 0.0}):
        ctx.expect_contact(
            frame,
            deck,
            elem_a="rear_hinge_pin",
            elem_b="moving_hinge_knuckle_0",
            contact_tol=0.006,
            name="rear hinge pin is captured by deck knuckles",
        )
        ctx.expect_contact(
            deck,
            lip,
            elem_a="front_hinge_pin",
            elem_b="lip_hinge_knuckle_0",
            contact_tol=0.006,
            name="front hinge pin is captured by lip knuckles",
        )
        ctx.expect_contact(
            deck,
            lip,
            elem_a="front_hinge_pin",
            elem_b="lip_hinge_knuckle_1",
            contact_tol=0.006,
            name="second front hinge knuckle captures the pin",
        )
        ctx.expect_overlap(
            deck,
            frame,
            axes="y",
            elem_a="main_deck_plate",
            elem_b="rear_curb",
            min_overlap=1.8,
            name="deck spans the heavy dock opening width",
        )
        for idx in (0, 1):
            ctx.expect_contact(
                frame,
                deck,
                elem_a=f"ram_rod_{idx}",
                elem_b=f"ram_top_lug_{idx}",
                contact_tol=0.003,
                name=f"hydraulic rod {idx} seats into deck lift lug",
            )
            ctx.expect_contact(
                frame,
                deck,
                elem_a=f"ram_rod_{idx}",
                elem_b=f"ram_top_pin_{idx}",
                contact_tol=0.003,
                name=f"hydraulic rod {idx} is pinned at the deck lug",
            )

    rest_deck_aabb = ctx.part_world_aabb(deck)
    rest_lip_aabb = ctx.part_world_aabb(lip)
    with ctx.pose({deck_hinge: 0.30, lip_hinge: 0.0}):
        raised_deck_aabb = ctx.part_world_aabb(deck)
    with ctx.pose({deck_hinge: 0.0, lip_hinge: 0.50}):
        lowered_lip_aabb = ctx.part_world_aabb(lip)

    ctx.check(
        "deck adjustment raises the free end",
        rest_deck_aabb is not None
        and raised_deck_aabb is not None
        and raised_deck_aabb[1][2] > rest_deck_aabb[1][2] + 0.20,
        details=f"rest={rest_deck_aabb}, raised={raised_deck_aabb}",
    )
    ctx.check(
        "lip rotates downward for truck-bed bridging",
        rest_lip_aabb is not None
        and lowered_lip_aabb is not None
        and lowered_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.20,
        details=f"rest={rest_lip_aabb}, lowered={lowered_lip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
