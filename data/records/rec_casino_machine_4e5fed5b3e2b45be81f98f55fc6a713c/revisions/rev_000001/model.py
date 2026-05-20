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
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arched_top_box(width: float, depth: float, base_z: float, shoulder_z: float, crown_z: float) -> MeshGeometry:
    """Closed extruded mesh for the rounded top box of the cabinet."""
    half_w = width / 2.0
    half_d = depth / 2.0

    profile: list[tuple[float, float]] = [(-half_w, base_z), (half_w, base_z), (half_w, shoulder_z)]
    for i in range(1, 16):
        theta = (i / 15.0) * math.pi
        # Half-ellipse crown from right shoulder to left shoulder.
        profile.append((half_w * math.cos(theta), shoulder_z + (crown_z - shoulder_z) * math.sin(theta)))
    profile.append((-half_w, base_z))

    geom = MeshGeometry()
    front: list[int] = []
    back: list[int] = []
    for x, z in profile:
        front.append(geom.add_vertex(x, -half_d, z))
    for x, z in profile:
        back.append(geom.add_vertex(x, half_d, z))

    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], front[j], back[j])
        geom.add_face(front[i], back[j], back[i])

    # Cap the front and back with a fan around the profile center.
    center_z = (base_z + crown_z) / 2.0
    front_c = geom.add_vertex(0.0, -half_d, center_z)
    back_c = geom.add_vertex(0.0, half_d, center_z)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front_c, front[i], front[j])
        geom.add_face(back_c, back[j], back[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_reel_slot_machine")

    cabinet_red = model.material("deep_casino_red", rgba=(0.55, 0.03, 0.04, 1.0))
    black = model.material("gloss_black", rgba=(0.005, 0.005, 0.006, 1.0))
    dark = model.material("smoked_glass", rgba=(0.05, 0.09, 0.13, 0.45))
    chrome = model.material("brushed_chrome", rgba=(0.78, 0.72, 0.60, 1.0))
    warm_gold = model.material("warm_gold", rgba=(0.95, 0.66, 0.18, 1.0))
    reel_paper = model.material("aged_reel_paper", rgba=(0.96, 0.91, 0.78, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.02, 0.018, 0.016, 1.0))
    service_blue = model.material("recessed_blue_door", rgba=(0.06, 0.12, 0.19, 1.0))

    cabinet = model.part("cabinet")

    # Full-height carcass panels. The front is intentionally built as rails and
    # surrounds so the reel window and service door read as real openings rather
    # than flat decals.
    cabinet.visual(Box((0.72, 0.56, 0.075)), origin=Origin(xyz=(0.0, 0.0, 0.0375)), material=black, name="floor_plinth")
    cabinet.visual(Box((0.05, 0.56, 1.48)), origin=Origin(xyz=(-0.335, 0.0, 0.79)), material=cabinet_red, name="side_panel_0")
    cabinet.visual(Box((0.05, 0.56, 1.48)), origin=Origin(xyz=(0.335, 0.0, 0.79)), material=cabinet_red, name="side_panel_1")
    cabinet.visual(Box((0.72, 0.05, 1.48)), origin=Origin(xyz=(0.0, 0.255, 0.79)), material=cabinet_red, name="rear_panel")
    cabinet.visual(Box((0.72, 0.56, 0.055)), origin=Origin(xyz=(0.0, 0.0, 1.345)), material=cabinet_red, name="top_tie")

    top_mesh = mesh_from_geometry(_arched_top_box(0.72, 0.56, 1.35, 1.62, 1.80), "curved_top_box")
    cabinet.visual(top_mesh, material=cabinet_red, name="curved_top_box")
    cabinet.visual(Box((0.50, 0.012, 0.16)), origin=Origin(xyz=(0.0, -0.286, 1.535)), material=warm_gold, name="top_marquee")
    cabinet.visual(Box((0.56, 0.014, 0.025)), origin=Origin(xyz=(0.0, -0.287, 1.43)), material=chrome, name="marquee_trim")

    # Reel window surround with a real open center.
    cabinet.visual(Box((0.055, 0.032, 0.46)), origin=Origin(xyz=(-0.3125, -0.296, 1.11)), material=chrome, name="reel_frame_left")
    cabinet.visual(Box((0.055, 0.032, 0.46)), origin=Origin(xyz=(0.3125, -0.296, 1.11)), material=chrome, name="reel_frame_right")
    cabinet.visual(Box((0.68, 0.032, 0.045)), origin=Origin(xyz=(0.0, -0.296, 1.3325)), material=chrome, name="reel_frame_top")
    cabinet.visual(Box((0.68, 0.032, 0.045)), origin=Origin(xyz=(0.0, -0.296, 0.8875)), material=chrome, name="reel_frame_bottom")
    cabinet.visual(Box((0.585, 0.006, 0.365)), origin=Origin(xyz=(0.0, -0.313, 1.11)), material=dark, name="reel_glass")

    # Lower service-door surround. The door itself is a child part recessed
    # behind this front plane.
    cabinet.visual(Box((0.065, 0.032, 0.64)), origin=Origin(xyz=(-0.3025, -0.296, 0.43)), material=chrome, name="door_jamb_0")
    cabinet.visual(Box((0.065, 0.032, 0.64)), origin=Origin(xyz=(0.3025, -0.296, 0.43)), material=chrome, name="door_jamb_1")
    cabinet.visual(Box((0.68, 0.032, 0.055)), origin=Origin(xyz=(0.0, -0.296, 0.105)), material=chrome, name="door_sill")
    cabinet.visual(Box((0.68, 0.032, 0.055)), origin=Origin(xyz=(0.0, -0.296, 0.735)), material=chrome, name="door_header")
    cabinet.visual(Box((0.011, 0.024, 0.54)), origin=Origin(xyz=(-0.2645, -0.291, 0.42)), material=chrome, name="hinge_leaf")

    # Projecting front shelf with a real ledge, front skirt, and under-braces.
    cabinet.visual(Box((0.68, 0.26, 0.055)), origin=Origin(xyz=(0.0, -0.41, 0.7625)), material=black, name="button_shelf")
    cabinet.visual(Box((0.68, 0.035, 0.105)), origin=Origin(xyz=(0.0, -0.535, 0.7125)), material=black, name="shelf_front_lip")
    cabinet.visual(Box((0.035, 0.19, 0.10)), origin=Origin(xyz=(-0.305, -0.39, 0.695)), material=rubber, name="shelf_brace_0")
    cabinet.visual(Box((0.035, 0.19, 0.10)), origin=Origin(xyz=(0.305, -0.39, 0.695)), material=rubber, name="shelf_brace_1")

    # Button sockets mounted in the ledge.
    button_positions = [-0.24, -0.12, 0.0, 0.12, 0.24]
    button_colors = [
        Material("button_red", rgba=(0.88, 0.03, 0.02, 1.0)),
        Material("button_yellow", rgba=(1.0, 0.78, 0.05, 1.0)),
        Material("button_green", rgba=(0.02, 0.62, 0.16, 1.0)),
        Material("button_blue", rgba=(0.04, 0.20, 0.86, 1.0)),
        Material("button_orange", rgba=(1.0, 0.34, 0.04, 1.0)),
    ]
    socket_top_z = 0.795
    for idx, x in enumerate(button_positions):
        cabinet.visual(
            Cylinder(radius=0.050, length=0.010),
            origin=Origin(xyz=(x, -0.42, socket_top_z - 0.005)),
            material=chrome,
            name=f"button_socket_{idx}",
        )
        button = model.part(f"button_{idx}")
        button.visual(
            Cylinder(radius=0.038, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=button_colors[idx],
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.028, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.030)),
            material=Material(f"button_highlight_{idx}", rgba=(1.0, 1.0, 1.0, 0.35)),
            name="top_highlight",
        )
        model.articulation(
            f"button_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, -0.42, socket_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.014),
        )

    # Three independently spinning mechanical reels visible through the window.
    reel_symbol_materials = [
        model.material("symbol_red", rgba=(0.85, 0.02, 0.02, 1.0)),
        model.material("symbol_green", rgba=(0.02, 0.55, 0.12, 1.0)),
        model.material("symbol_blue", rgba=(0.02, 0.14, 0.75, 1.0)),
        model.material("symbol_gold", rgba=(0.94, 0.65, 0.05, 1.0)),
    ]
    for idx, x in enumerate((-0.18, 0.0, 0.18)):
        cabinet.visual(
            Box((0.018, 0.50, 0.045)),
            origin=Origin(xyz=(x - 0.075, 0.015, 1.0755)),
            material=chrome,
            name=f"reel_bearing_{idx}_0",
        )
        cabinet.visual(
            Box((0.018, 0.50, 0.045)),
            origin=Origin(xyz=(x + 0.075, 0.015, 1.0755)),
            material=chrome,
            name=f"reel_bearing_{idx}_1",
        )
        reel = model.part(f"reel_{idx}")
        reel.visual(
            Cylinder(radius=0.125, length=0.118),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=reel_paper,
            name="drum",
        )
        reel.visual(
            Cylinder(radius=0.012, length=0.170),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="axle",
        )
        for s_idx, z in enumerate((-0.075, 0.0, 0.075)):
            reel.visual(
                Box((0.090, 0.004, 0.040)),
                origin=Origin(xyz=(0.0, -math.sqrt(0.125 * 0.125 - z * z) - 0.002, z)),
                material=reel_symbol_materials[(idx + s_idx) % len(reel_symbol_materials)],
                name=f"symbol_{s_idx}",
            )
        model.articulation(
            f"reel_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=reel,
            origin=Origin(xyz=(x, -0.155, 1.11)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    # Recessed lower service door, carried on a vertical side hinge.
    door = model.part("service_door")
    door.visual(
        Box((0.49, 0.022, 0.52)),
        origin=Origin(xyz=(0.245, 0.011, 0.0)),
        material=service_blue,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.54),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.055, 0.018, 0.18)),
        origin=Origin(xyz=(0.38, -0.007, 0.06)),
        material=chrome,
        name="pull_handle",
    )
    door.visual(
        Box((0.20, 0.004, 0.040)),
        origin=Origin(xyz=(0.245, -0.001, 0.17)),
        material=black,
        name="service_label",
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.245, -0.302, 0.42)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("service_door")
    hinge = object_model.get_articulation("door_hinge")

    frame_aabb = ctx.part_element_world_aabb(cabinet, elem="door_header")
    door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    shelf_aabb = ctx.part_element_world_aabb(cabinet, elem="button_shelf")
    rest_door_aabb = ctx.part_element_world_aabb(door, elem="pull_handle")

    ctx.check(
        "service door front is recessed behind cabinet frame",
        frame_aabb is not None
        and door_aabb is not None
        and door_aabb[0][1] > frame_aabb[0][1] + 0.005,
        details=f"frame_aabb={frame_aabb}, door_aabb={door_aabb}",
    )
    ctx.check(
        "button shelf projects as a ledge",
        shelf_aabb is not None and shelf_aabb[0][1] < -0.50 and shelf_aabb[1][1] >= -0.282,
        details=f"shelf_aabb={shelf_aabb}",
    )

    with ctx.pose({hinge: 1.2}):
        opened_handle_aabb = ctx.part_element_world_aabb(door, elem="pull_handle")
    ctx.check(
        "service door swings outward on side hinge",
        rest_door_aabb is not None
        and opened_handle_aabb is not None
        and opened_handle_aabb[0][1] < rest_door_aabb[0][1] - 0.10,
        details=f"rest={rest_door_aabb}, opened={opened_handle_aabb}",
    )

    for idx in range(5):
        button = object_model.get_part(f"button_{idx}")
        slide = object_model.get_articulation(f"button_{idx}_slide")
        ctx.expect_gap(
            button,
            cabinet,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="cap",
            negative_elem=f"button_socket_{idx}",
            name=f"button_{idx} sits on its socket",
        )
        ctx.expect_overlap(
            button,
            cabinet,
            axes="xy",
            min_overlap=0.025,
            elem_a="cap",
            elem_b=f"button_socket_{idx}",
            name=f"button_{idx} is centered in its socket",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({slide: 0.012}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} pushes downward independently",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.010,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    for idx in range(3):
        reel = object_model.get_part(f"reel_{idx}")
        ctx.expect_within(
            reel,
            cabinet,
            axes="xz",
            margin=0.02,
            inner_elem="drum",
            outer_elem="reel_glass",
            name=f"reel_{idx} stays framed by the window",
        )

    return ctx.report()


object_model = build_object_model()
