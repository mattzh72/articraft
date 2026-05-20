from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_padmount_transformer")

    steel_green = model.material("weathered_olive_green", rgba=(0.20, 0.38, 0.22, 1.0))
    dark_green = model.material("dark_base_green", rgba=(0.08, 0.17, 0.10, 1.0))
    shadow = model.material("shadow_black", rgba=(0.02, 0.025, 0.02, 1.0))
    hinge_metal = model.material("galvanized_hinge_pin", rgba=(0.58, 0.62, 0.58, 1.0))
    inner_gray = model.material("powdercoat_inner_gray", rgba=(0.72, 0.74, 0.70, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(1.0, 0.78, 0.10, 1.0))
    warning_red = model.material("warning_red", rgba=(0.78, 0.08, 0.06, 1.0))

    housing = model.part("housing")

    def add_housing_box(name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material: Material) -> None:
        housing.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # A broad, hollow steel enclosure: rear/side/top/bottom panels plus the front
    # frame around the double-door opening.  Dimensions are in the range of a
    # real residential padmount transformer cabinet.
    width = 2.40
    depth = 0.92
    shell_height = 1.42
    floor_z = 0.12
    shell_center_z = floor_z + shell_height / 2.0
    front_y = -depth / 2.0
    rear_y = depth / 2.0

    add_housing_box("concrete_plinth", (2.55, 1.05, 0.12), (0.0, 0.0, 0.06), dark_green)
    add_housing_box("floor_pan", (width, depth, 0.06), (0.0, 0.0, floor_z + 0.03), steel_green)
    add_housing_box("rear_wall", (width, 0.055, shell_height), (0.0, rear_y - 0.0275, shell_center_z), steel_green)
    add_housing_box("side_wall_0", (0.065, depth, shell_height), (-width / 2.0 + 0.0325, 0.0, shell_center_z), steel_green)
    add_housing_box("side_wall_1", (0.065, depth, shell_height), (width / 2.0 - 0.0325, 0.0, shell_center_z), steel_green)
    add_housing_box("roof_cap", (2.54, 1.04, 0.085), (0.0, 0.0, floor_z + shell_height + 0.0425), steel_green)
    add_housing_box("front_sill", (width, 0.075, 0.095), (0.0, front_y - 0.0125, floor_z + 0.0475), steel_green)
    add_housing_box("front_lintel", (width, 0.075, 0.105), (0.0, front_y - 0.0125, floor_z + shell_height - 0.0525), steel_green)
    add_housing_box("front_frame_0", (0.085, 0.075, shell_height), (-width / 2.0 + 0.0425, front_y - 0.0125, shell_center_z), steel_green)
    add_housing_box("front_frame_1", (0.085, 0.075, shell_height), (width / 2.0 - 0.0425, front_y - 0.0125, shell_center_z), steel_green)
    add_housing_box("center_mullion", (0.080, 0.075, shell_height), (0.0, front_y - 0.0125, shell_center_z), steel_green)

    # Side ventilation louvers and small lifting lugs are fixed to the housing.
    for side_x, side_name in [(-width / 2.0 - 0.006, "0"), (width / 2.0 + 0.006, "1")]:
        for i in range(5):
            add_housing_box(
                f"side_louver_{side_name}_{i}",
                (0.018, 0.36, 0.025),
                (side_x, 0.08, 0.47 + i * 0.06),
                shadow,
            )
    add_housing_box("lifting_lug_0", (0.18, 0.055, 0.055), (-0.78, -0.28, floor_z + shell_height + 0.11), hinge_metal)
    add_housing_box("lifting_lug_1", (0.18, 0.055, 0.055), (0.78, -0.28, floor_z + shell_height + 0.11), hinge_metal)

    door_width = 1.08
    door_height = 1.30
    door_thickness = 0.045
    door_bottom_z = floor_z + 0.095
    # Place the hinge pin line just outside the front frame: the 25 mm hinge
    # barrels kiss the steel frame, while the flat door skin retains a small
    # clearance from the painted frame face.
    hinge_y = front_y - 0.074
    hinge_x_0 = -width / 2.0 + 0.090
    hinge_x_1 = width / 2.0 - 0.090

    def build_door(index: int, hinge_x: float, extends_positive_x: bool):
        door = model.part(f"door_{index}")
        sign = 1.0 if extends_positive_x else -1.0
        panel_center_x = sign * door_width / 2.0

        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(panel_center_x, 0.0, door_height / 2.0)),
            material=steel_green,
            name="outer_skin",
        )
        # Raised folded-steel door perimeter and central stiffener, slightly
        # embedded into the skin so each door reads as one welded assembly.
        rib_y = -door_thickness / 2.0 - 0.004
        for rib_name, x in [
            ("hinge_stile", sign * 0.030),
            ("latch_stile", sign * (door_width - 0.030)),
            ("center_stiffener", sign * (door_width * 0.52)),
        ]:
            door.visual(
                Box((0.055, 0.014, door_height - 0.08)),
                origin=Origin(xyz=(x, rib_y, door_height / 2.0)),
                material=steel_green,
                name=rib_name,
            )
        for rib_name, z in [("bottom_rail", 0.045), ("top_rail", door_height - 0.045)]:
            door.visual(
                Box((door_width - 0.09, 0.014, 0.055)),
                origin=Origin(xyz=(sign * door_width / 2.0, rib_y, z)),
                material=steel_green,
                name=rib_name,
            )

        # Three exposed knuckles on the door leaf surround the revolute pin line.
        for i, z in enumerate((0.18, 0.60, 1.02)):
            door.visual(
                Cylinder(radius=0.025, length=0.22),
                origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, 0.0, 0.0)),
                material=hinge_metal,
                name=f"hinge_barrel_{i}",
            )
        door.visual(
            Cylinder(radius=0.010, length=door_height + 0.08),
            origin=Origin(xyz=(0.0, 0.0, door_height / 2.0), rpy=(0.0, 0.0, 0.0)),
            material=hinge_metal,
            name="hinge_pin",
        )
        # Padlock hasp and recessed pull on the inner meeting edge.
        handle_x = sign * (door_width - 0.17)
        door.visual(
            Box((0.105, 0.022, 0.055)),
            origin=Origin(xyz=(handle_x, -door_thickness / 2.0 - 0.009, 0.70)),
            material=shadow,
            name="latch_hasp",
        )
        door.visual(
            Box((0.040, 0.030, 0.150)),
            origin=Origin(xyz=(handle_x + sign * 0.060, -door_thickness / 2.0 - 0.011, 0.69)),
            material=hinge_metal,
            name="pull_handle",
        )

        label_x = sign * (door_width * 0.36)
        door.visual(
            Box((0.22, 0.006, 0.12)),
            origin=Origin(xyz=(label_x, -door_thickness / 2.0 - 0.002, 0.98)),
            material=warning_yellow,
            name="front_warning_label",
        )
        door.visual(
            Box((0.060, 0.007, 0.055)),
            origin=Origin(xyz=(label_x, -door_thickness / 2.0 - 0.003, 0.98)),
            material=warning_red,
            name="warning_symbol",
        )

        model.articulation(
            f"housing_to_door_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=door,
            origin=Origin(xyz=(hinge_x, hinge_y, door_bottom_z)),
            axis=(0.0, 0.0, -sign),
            motion_limits=MotionLimits(effort=85.0, velocity=1.2, lower=0.0, upper=1.95),
        )

        live_panel = model.part(f"live_panel_{index}")
        live_width = 0.82
        live_height = 1.02
        live_thickness = 0.026
        live_panel.visual(
            Box((live_width, live_thickness, live_height)),
            origin=Origin(xyz=(sign * live_width / 2.0, 0.0, live_height / 2.0)),
            material=inner_gray,
            name="deadfront_sheet",
        )
        # Folded lips make the inner live-side shield panel read as a real
        # hinged steel cover, not a flat card.
        lip_y = live_thickness / 2.0 + 0.004
        live_panel.visual(
            Box((0.045, 0.016, live_height)),
            origin=Origin(xyz=(sign * 0.0225, lip_y, live_height / 2.0)),
            material=inner_gray,
            name="hinge_fold",
        )
        live_panel.visual(
            Box((0.045, 0.016, live_height)),
            origin=Origin(xyz=(sign * (live_width - 0.0225), lip_y, live_height / 2.0)),
            material=inner_gray,
            name="free_edge_fold",
        )
        live_panel.visual(
            Box((live_width - 0.05, 0.016, 0.045)),
            origin=Origin(xyz=(sign * live_width / 2.0, lip_y, live_height - 0.0225)),
            material=inner_gray,
            name="top_fold",
        )
        live_panel.visual(
            Box((live_width - 0.05, 0.016, 0.045)),
            origin=Origin(xyz=(sign * live_width / 2.0, lip_y, 0.0225)),
            material=inner_gray,
            name="bottom_fold",
        )
        live_panel.visual(
            Cylinder(radius=0.014, length=live_height + 0.04),
            origin=Origin(xyz=(0.0, 0.0, live_height / 2.0)),
            material=hinge_metal,
            name="secondary_hinge_pin",
        )
        live_panel.visual(
            Box((0.30, 0.006, 0.16)),
            origin=Origin(xyz=(sign * live_width * 0.52, live_thickness / 2.0 + 0.002, live_height * 0.68)),
            material=warning_yellow,
            name="live_side_warning",
        )
        live_panel.visual(
            Box((0.075, 0.007, 0.065)),
            origin=Origin(xyz=(sign * live_width * 0.52, live_thickness / 2.0 + 0.002, live_height * 0.68)),
            material=warning_red,
            name="voltage_symbol",
        )
        for i, z in enumerate((0.18, 0.50, 0.82)):
            live_panel.visual(
                Cylinder(radius=0.020, length=0.012),
                origin=Origin(xyz=(sign * (live_width - 0.10), live_thickness / 2.0 + 0.001, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=hinge_metal,
                name=f"quarter_turn_latch_{i}",
            )

        # The inner shield is mounted just behind the outer door skin, lower and
        # narrower than the outer door so it clears the frame when closed.
        secondary_hinge_x = sign * 0.13
        secondary_hinge_y = 0.070
        secondary_hinge_z = 0.18
        model.articulation(
            f"door_{index}_to_live_panel",
            ArticulationType.REVOLUTE,
            parent=door,
            child=live_panel,
            origin=Origin(xyz=(secondary_hinge_x, secondary_hinge_y, secondary_hinge_z)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=1.35),
        )
        return door, live_panel

    build_door(0, hinge_x_0, True)
    build_door(1, hinge_x_1, False)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    live_panel_0 = object_model.get_part("live_panel_0")
    live_panel_1 = object_model.get_part("live_panel_1")
    hinge_0 = object_model.get_articulation("housing_to_door_0")
    hinge_1 = object_model.get_articulation("housing_to_door_1")
    inner_hinge_0 = object_model.get_articulation("door_0_to_live_panel")
    inner_hinge_1 = object_model.get_articulation("door_1_to_live_panel")

    # The exposed hinge knuckles are deliberately seated a millimeter into the
    # front frame leaf so the heavy steel doors have a visible load path instead
    # of floating just in front of the cabinet.
    for door, frame_elem in [(door_0, "front_frame_0"), (door_1, "front_frame_1")]:
        for i in range(3):
            barrel = f"hinge_barrel_{i}"
            ctx.allow_overlap(
                housing,
                door,
                elem_a=frame_elem,
                elem_b=barrel,
                reason="Exposed hinge barrel is captured against the welded outer frame leaf.",
            )
            ctx.expect_gap(
                housing,
                door,
                axis="y",
                positive_elem=frame_elem,
                negative_elem=barrel,
                max_gap=0.002,
                max_penetration=0.003,
                name=f"{door.name} {barrel} seated in frame leaf",
            )
            ctx.expect_overlap(
                housing,
                door,
                axes="xz",
                elem_a=frame_elem,
                elem_b=barrel,
                min_overlap=0.015,
                name=f"{door.name} {barrel} aligns with vertical frame",
            )

    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0, inner_hinge_0: 0.0, inner_hinge_1: 0.0}):
        for door, live_panel in [(door_0, live_panel_0), (door_1, live_panel_1)]:
            ctx.expect_gap(
                live_panel,
                door,
                axis="y",
                positive_elem="deadfront_sheet",
                negative_elem="outer_skin",
                min_gap=0.020,
                max_gap=0.060,
                name=f"{live_panel.name} stands inside {door.name}",
            )
            ctx.expect_within(
                live_panel,
                door,
                axes="xz",
                inner_elem="deadfront_sheet",
                outer_elem="outer_skin",
                margin=0.005,
                name=f"{live_panel.name} is smaller than its outer door",
            )

    rest_aabb_0 = ctx.part_world_aabb(door_0)
    rest_aabb_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({hinge_0: 1.20, hinge_1: 1.20}):
        open_aabb_0 = ctx.part_world_aabb(door_0)
        open_aabb_1 = ctx.part_world_aabb(door_1)
    ctx.check(
        "front doors swing outward",
        rest_aabb_0 is not None
        and rest_aabb_1 is not None
        and open_aabb_0 is not None
        and open_aabb_1 is not None
        and open_aabb_0[0][1] < rest_aabb_0[0][1] - 0.25
        and open_aabb_1[0][1] < rest_aabb_1[0][1] - 0.25,
        details=f"rest={rest_aabb_0}, {rest_aabb_1}; open={open_aabb_0}, {open_aabb_1}",
    )

    with ctx.pose({hinge_0: 1.35, hinge_1: 1.35, inner_hinge_0: 0.0, inner_hinge_1: 0.0}):
        panel_rest_0 = ctx.part_world_aabb(live_panel_0)
        panel_rest_1 = ctx.part_world_aabb(live_panel_1)
    with ctx.pose({hinge_0: 1.35, hinge_1: 1.35, inner_hinge_0: 1.0, inner_hinge_1: 1.0}):
        panel_open_0 = ctx.part_world_aabb(live_panel_0)
        panel_open_1 = ctx.part_world_aabb(live_panel_1)
    ctx.check(
        "inner live panels have secondary swing",
        panel_rest_0 is not None
        and panel_rest_1 is not None
        and panel_open_0 is not None
        and panel_open_1 is not None
        and abs(panel_open_0[1][0] - panel_rest_0[1][0]) > 0.12
        and abs(panel_open_1[0][0] - panel_rest_1[0][0]) > 0.12,
        details=f"rest={panel_rest_0}, {panel_rest_1}; open={panel_open_0}, {panel_open_1}",
    )

    return ctx.report()


object_model = build_object_model()
