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


def _prism_from_yz_profile(
    profile: list[tuple[float, float]], width: float, *, x_center: float = 0.0
) -> MeshGeometry:
    """Build a closed prism by extruding a side-profile along cabinet width."""

    geom = MeshGeometry()
    x0 = x_center - width / 2.0
    x1 = x_center + width / 2.0
    for x in (x0, x1):
        for y, z in profile:
            geom.add_vertex(x, y, z)

    n = len(profile)
    # End caps, fan triangulated.  Face winding is not semantically important here,
    # but keeping both caps opposite makes the generated mesh well behaved.
    for i in range(1, n - 1):
        geom.add_face(0, i, i + 1)
        geom.add_face(n, n + i + 1, n + i)

    # Side walls.
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(i, j, n + j)
        geom.add_face(i, n + j, n + i)
    return geom


def _angled_origin(
    center_y: float,
    center_z: float,
    roll: float,
    *,
    local_y: float = 0.0,
    local_z: float = 0.0,
    x: float = 0.0,
) -> Origin:
    """Offset a visual in the local coordinates of the slanted display face."""

    c = math.cos(roll)
    s = math.sin(roll)
    return Origin(
        xyz=(
            x,
            center_y + c * local_y - s * local_z,
            center_z + s * local_y + c * local_z,
        ),
        rpy=(roll, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slant_top_video_slot_machine")

    red = model.material("deep_red_laminate", rgba=(0.42, 0.025, 0.035, 1.0))
    black = model.material("black_powdercoat", rgba=(0.015, 0.015, 0.018, 1.0))
    charcoal = model.material("charcoal_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.70, 0.65, 1.0))
    glass = model.material("dark_blue_glass", rgba=(0.015, 0.035, 0.085, 0.92))
    glow = model.material("lit_marquee", rgba=(1.0, 0.83, 0.30, 1.0))
    white = model.material("white_legends", rgba=(0.92, 0.93, 0.88, 1.0))
    gold = model.material("coin_gold", rgba=(0.95, 0.68, 0.20, 1.0))

    cabinet = model.part("cabinet")

    # Casino-floor sized slant-top cabinet: about 0.8 m wide and 1.5 m tall.
    # The side polygon creates a continuous wedge-like body with a reclined
    # display face above the projecting button deck.
    shell_profile = [
        (-0.34, 0.00),
        (0.30, 0.00),
        (0.30, 1.46),
        (0.10, 1.52),
        (-0.12, 1.38),
        (-0.37, 0.92),
        (-0.37, 0.18),
    ]
    cabinet.visual(
        mesh_from_geometry(_prism_from_yz_profile(shell_profile, 0.82), "cabinet_shell"),
        material=red,
        name="cabinet_shell",
    )

    cabinet.visual(
        Box((0.88, 0.72, 0.12)),
        origin=Origin(xyz=(0.0, -0.02, 0.06)),
        material=black,
        name="floor_plinth",
    )

    # Shallow shelf that projects from the cabinet front and is carried by real
    # triangular side gussets, not by floating buttons.
    cabinet.visual(
        Box((0.76, 0.31, 0.065)),
        origin=Origin(xyz=(0.0, -0.52, 0.785)),
        material=charcoal,
        name="button_deck",
    )
    cabinet.visual(
        Cylinder(radius=0.025, length=0.76),
        origin=Origin(xyz=(0.0, -0.675, 0.785), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="deck_front_roll",
    )
    bracket_profile = [
        (-0.374, 0.758),
        (-0.646, 0.758),
        (-0.374, 0.58),
    ]
    for x, suffix in [(-0.39, "0"), (0.39, "1")]:
        cabinet.visual(
            mesh_from_geometry(
                _prism_from_yz_profile(bracket_profile, 0.045, x_center=x),
                f"deck_gusset_{suffix}",
            ),
            material=charcoal,
            name=f"deck_gusset_{suffix}",
        )

    # Slanted video face, set into the angled body.  Thin trim pieces are
    # slightly embedded in the same plane to read as a manufactured bezel.
    face_roll = -0.498
    face_y = -0.245
    face_z = 1.15
    screen_w = 0.60
    screen_h = 0.36
    trim = 0.035
    cabinet.visual(
        Box((screen_w, 0.018, screen_h)),
        origin=_angled_origin(face_y, face_z, face_roll),
        material=glass,
        name="video_screen",
    )
    cabinet.visual(
        Box((screen_w + 2 * trim, 0.026, trim)),
        origin=_angled_origin(face_y, face_z, face_roll, local_z=screen_h / 2 + trim / 2),
        material=black,
        name="screen_top_bezel",
    )
    cabinet.visual(
        Box((screen_w + 2 * trim, 0.026, trim)),
        origin=_angled_origin(face_y, face_z, face_roll, local_z=-(screen_h / 2 + trim / 2)),
        material=black,
        name="screen_bottom_bezel",
    )
    for x, side_name in [(-(screen_w / 2 + trim / 2), "screen_side_bezel_0"), ((screen_w / 2 + trim / 2), "screen_side_bezel_1")]:
        cabinet.visual(
            Box((trim, 0.026, screen_h)),
            origin=_angled_origin(face_y, face_z, face_roll, x=x),
            material=black,
            name=side_name,
        )
    cabinet.visual(
        Box((0.50, 0.010, 0.035)),
        origin=_angled_origin(face_y, face_z, face_roll, local_z=-0.09, local_y=-0.012),
        material=glow,
        name="game_graphic",
    )
    cabinet.visual(
        Box((0.68, 0.030, 0.13)),
        origin=_angled_origin(-0.09, 1.41, face_roll, local_y=-0.006),
        material=glow,
        name="top_marquee",
    )

    # Belly door surround, bill/card service details and hinge-side support.
    cabinet.visual(
        Box((0.73, 0.020, 0.055)),
        origin=Origin(xyz=(0.0, -0.382, 0.735)),
        material=chrome,
        name="belly_header_trim",
    )
    cabinet.visual(
        Box((0.030, 0.020, 0.56)),
        origin=Origin(xyz=(-0.345, -0.374, 0.46)),
        material=chrome,
        name="hinge_jamb",
    )
    cabinet.visual(
        Box((0.030, 0.020, 0.56)),
        origin=Origin(xyz=(0.285, -0.382, 0.46)),
        material=chrome,
        name="latch_jamb",
    )
    cabinet.visual(
        Box((0.30, 0.020, 0.075)),
        origin=Origin(xyz=(0.12, -0.386, 0.845)),
        material=black,
        name="bill_acceptor",
    )
    cabinet.visual(
        Box((0.16, 0.024, 0.030)),
        origin=Origin(xyz=(0.12, -0.398, 0.848)),
        material=chrome,
        name="bill_slot",
    )
    cabinet.visual(
        Box((0.12, 0.018, 0.055)),
        origin=Origin(xyz=(-0.20, -0.387, 0.842)),
        material=black,
        name="card_reader",
    )

    # Fixed collars in the deck make the row of play buttons visibly mounted.
    button_xs = [-0.27, -0.162, -0.054, 0.054, 0.162, 0.27]
    button_y = -0.585
    collar_top_z = 0.824
    for idx, x in enumerate(button_xs):
        cabinet.visual(
            Cylinder(radius=0.044, length=0.0065),
            origin=Origin(xyz=(x, button_y, collar_top_z - 0.0065 / 2.0)),
            material=black,
            name=f"button_collar_{idx}",
        )

    # The belly door is a separate hinged part so it can swing open from one side.
    belly_door = model.part("belly_door")
    belly_door.visual(
        Box((0.62, 0.032, 0.52)),
        origin=Origin(xyz=(0.31, -0.008, 0.26)),
        material=charcoal,
        name="door_panel",
    )
    belly_door.visual(
        Box((0.54, 0.006, 0.065)),
        origin=Origin(xyz=(0.33, -0.027, 0.39)),
        material=black,
        name="door_vent",
    )
    belly_door.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.50, -0.030, 0.31), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lock_cylinder",
    )
    belly_door.visual(
        Box((0.030, 0.012, 0.004)),
        origin=Origin(xyz=(0.50, -0.042, 0.31)),
        material=black,
        name="lock_slot",
    )
    for z in (0.12, 0.40):
        belly_door.visual(
            Cylinder(radius=0.010, length=0.16),
            origin=Origin(xyz=(0.004, -0.010, z)),
            material=chrome,
            name=f"hinge_barrel_{int(z * 100)}",
        )

    door_hinge = model.articulation(
        "belly_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=belly_door,
        origin=Origin(xyz=(-0.36, -0.40, 0.20)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    _ = door_hinge

    button_materials = [
        model.material("button_red", rgba=(0.86, 0.05, 0.03, 1.0)),
        model.material("button_blue", rgba=(0.05, 0.22, 0.90, 1.0)),
        model.material("button_green", rgba=(0.04, 0.62, 0.16, 1.0)),
        model.material("button_yellow", rgba=(0.94, 0.82, 0.08, 1.0)),
        model.material("button_purple", rgba=(0.42, 0.12, 0.70, 1.0)),
        model.material("button_white", rgba=(0.92, 0.92, 0.86, 1.0)),
    ]
    for idx, x in enumerate(button_xs):
        button = model.part(f"button_{idx}")
        button.visual(
            Cylinder(radius=0.034, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=button_materials[idx],
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.018, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0275)),
            material=white if idx != 3 else gold,
            name="legend_lens",
        )
        model.articulation(
            f"button_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, button_y, collar_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.016),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("belly_door")
    hinge = object_model.get_articulation("belly_door_hinge")

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="cabinet_shell",
        negative_elem="door_panel",
        min_gap=0.005,
        max_gap=0.060,
        name="closed belly door sits proud of cabinet face",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="door_panel",
        elem_b="cabinet_shell",
        min_overlap=0.45,
        name="belly door covers the lower service opening",
    )

    deck_bounds = ctx.part_element_world_aabb(cabinet, elem="button_deck")
    shell_bounds = ctx.part_element_world_aabb(cabinet, elem="cabinet_shell")
    ctx.check(
        "button deck projects from the cabinet front",
        deck_bounds is not None
        and shell_bounds is not None
        and deck_bounds[0][1] < shell_bounds[0][1] - 0.20,
        details=f"deck_bounds={deck_bounds}, shell_bounds={shell_bounds}",
    )

    rest_door = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: 1.10}):
        open_door = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "belly door swings outward on a vertical side hinge",
        rest_door is not None
        and open_door is not None
        and open_door[0][1] < rest_door[0][1] - 0.18,
        details=f"closed={rest_door}, opened={open_door}",
    )

    button_centers = []
    for idx in range(6):
        button = object_model.get_part(f"button_{idx}")
        slide = object_model.get_articulation(f"button_{idx}_slide")
        ctx.expect_contact(
            button,
            cabinet,
            elem_a="cap",
            elem_b=f"button_collar_{idx}",
            contact_tol=0.002,
            name=f"button {idx} is seated in its deck collar",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({slide: 0.016}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {idx} depresses independently",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.012,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )
        cap_bounds = ctx.part_element_world_aabb(button, elem="cap")
        if cap_bounds is not None:
            button_centers.append(
                (
                    (cap_bounds[0][0] + cap_bounds[1][0]) / 2.0,
                    (cap_bounds[0][1] + cap_bounds[1][1]) / 2.0,
                )
            )

    ctx.check(
        "play buttons form one row along the deck leading edge",
        len(button_centers) == 6
        and all(button_centers[i][0] < button_centers[i + 1][0] for i in range(5))
        and deck_bounds is not None
        and all(abs(center_y - (deck_bounds[0][1] + 0.09)) < 0.035 for _, center_y in button_centers),
        details=f"button_centers={button_centers}, deck_bounds={deck_bounds}",
    )

    return ctx.report()


object_model = build_object_model()
