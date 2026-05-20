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
    mesh_from_cadquery,
)
import cadquery as cq


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _housing_shell() -> cq.Workplane:
    """Connected sheet-metal duct purifier housing with open rear service aperture."""

    depth = 0.72
    width = 1.20
    height = 0.46
    wall = 0.035

    parts: list[cq.Workplane] = []

    # Main rectangular split-case shell, open through the duct axis.
    parts.append(_cq_box((depth, width, wall), (0.0, 0.0, height / 2 - wall / 2)))
    parts.append(_cq_box((depth, width, wall), (0.0, 0.0, -height / 2 + wall / 2)))
    parts.append(_cq_box((depth, wall, height), (0.0, width / 2 - wall / 2, 0.0)))
    parts.append(_cq_box((depth, wall, height), (0.0, -width / 2 + wall / 2, 0.0)))

    # Front reducer/flange ring and rectangular duct collar.
    front_x = depth / 2 + 0.001
    duct_w = 0.86
    duct_h = 0.28
    ring_t = 0.030
    parts.append(_cq_box((ring_t, width, (height - duct_h) / 2), (front_x, 0.0, (height + duct_h) / 4)))
    parts.append(_cq_box((ring_t, width, (height - duct_h) / 2), (front_x, 0.0, -(height + duct_h) / 4)))
    parts.append(_cq_box((ring_t, (width - duct_w) / 2, duct_h + 0.004), (front_x, (width + duct_w) / 4, 0.0)))
    parts.append(_cq_box((ring_t, (width - duct_w) / 2, duct_h + 0.004), (front_x, -(width + duct_w) / 4, 0.0)))

    sleeve_len = 0.15
    sleeve_x = depth / 2 + sleeve_len / 2
    sleeve_wall = 0.026
    parts.append(_cq_box((sleeve_len, duct_w, sleeve_wall), (sleeve_x, 0.0, duct_h / 2 - sleeve_wall / 2)))
    parts.append(_cq_box((sleeve_len, duct_w, sleeve_wall), (sleeve_x, 0.0, -duct_h / 2 + sleeve_wall / 2)))
    parts.append(_cq_box((sleeve_len, sleeve_wall, duct_h), (sleeve_x, duct_w / 2 - sleeve_wall / 2, 0.0)))
    parts.append(_cq_box((sleeve_len, sleeve_wall, duct_h), (sleeve_x, -duct_w / 2 + sleeve_wall / 2, 0.0)))

    # Thin front mounting flange around the duct collar.
    flange_x = depth / 2 + sleeve_len + 0.010
    flange_t = 0.018
    flange_w = 0.99
    flange_h = 0.39
    parts.append(_cq_box((flange_t, flange_w, 0.035), (flange_x, 0.0, flange_h / 2 - 0.0175)))
    parts.append(_cq_box((flange_t, flange_w, 0.035), (flange_x, 0.0, -flange_h / 2 + 0.0175)))
    parts.append(_cq_box((flange_t, 0.035, flange_h), (flange_x, flange_w / 2 - 0.0175, 0.0)))
    parts.append(_cq_box((flange_t, 0.035, flange_h), (flange_x, -flange_w / 2 + 0.0175, 0.0)))

    # Rear service opening trim, proud enough to read as a door seat.
    rear_x = -depth / 2 - 0.002
    open_w = 0.98
    open_h = 0.35
    lip_t = 0.024
    parts.append(_cq_box((lip_t, width, 0.050), (rear_x, 0.0, open_h / 2 + 0.025)))
    parts.append(_cq_box((lip_t, width, 0.050), (rear_x, 0.0, -open_h / 2 - 0.025)))
    parts.append(_cq_box((lip_t, (width - open_w) / 2, open_h + 0.004), (rear_x, (width + open_w) / 4, 0.0)))
    parts.append(_cq_box((lip_t, (width - open_w) / 2, open_h + 0.004), (rear_x, -(width + open_w) / 4, 0.0)))

    # Internal guide rails for the sliding filter cassette.
    rail_x = -0.125
    rail_len = 0.43
    parts.append(_cq_box((rail_len, 0.032, 0.022), (rail_x, 0.475, -0.155)))
    parts.append(_cq_box((rail_len, 0.032, 0.022), (rail_x, -0.475, -0.155)))
    parts.append(_cq_box((rail_len, 0.032, 0.018), (rail_x, 0.475, 0.155)))
    parts.append(_cq_box((rail_len, 0.032, 0.018), (rail_x, -0.475, 0.155)))

    shell = parts[0]
    for item in parts[1:]:
        shell = shell.union(item)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="duct_mounted_air_purifier")

    powder = model.material("warm_powder_coated_metal", rgba=(0.72, 0.75, 0.76, 1.0))
    dark = model.material("dark_rubber_and_handles", rgba=(0.02, 0.025, 0.028, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.50, 0.52, 0.53, 1.0))
    blue_plastic = model.material("blue_filter_frame", rgba=(0.08, 0.23, 0.42, 1.0))
    filter_media = model.material("pleated_white_filter_media", rgba=(0.92, 0.95, 0.93, 1.0))
    pale_blue = model.material("pale_blue_pleat_shadow", rgba=(0.56, 0.70, 0.82, 1.0))

    housing = model.part("housing")
    # Box-like sheet-metal housing, open through the duct axis and at the rear service door.
    housing.visual(Box((0.72, 1.20, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.2125)), material=powder, name="top_wall")
    housing.visual(Box((0.72, 1.20, 0.035)), origin=Origin(xyz=(0.0, 0.0, -0.2125)), material=powder, name="bottom_wall")
    housing.visual(Box((0.72, 0.045, 0.46)), origin=Origin(xyz=(0.0, -0.5775, 0.0)), material=powder, name="side_wall_0")
    housing.visual(Box((0.72, 0.045, 0.46)), origin=Origin(xyz=(0.0, 0.5775, 0.0)), material=powder, name="side_wall_1")
    # Front duct transition ring.
    housing.visual(Box((0.035, 1.20, 0.095)), origin=Origin(xyz=(0.362, 0.0, 0.1825)), material=powder, name="front_ring_top")
    housing.visual(Box((0.035, 1.20, 0.095)), origin=Origin(xyz=(0.362, 0.0, -0.1825)), material=powder, name="front_ring_bottom")
    housing.visual(Box((0.035, 0.190, 0.290)), origin=Origin(xyz=(0.362, -0.505, 0.0)), material=powder, name="front_ring_side_0")
    housing.visual(Box((0.035, 0.190, 0.290)), origin=Origin(xyz=(0.362, 0.505, 0.0)), material=powder, name="front_ring_side_1")
    housing.visual(Box((0.155, 0.86, 0.026)), origin=Origin(xyz=(0.435, 0.0, 0.127)), material=powder, name="duct_sleeve_top")
    housing.visual(Box((0.155, 0.86, 0.026)), origin=Origin(xyz=(0.435, 0.0, -0.127)), material=powder, name="duct_sleeve_bottom")
    housing.visual(Box((0.155, 0.026, 0.280)), origin=Origin(xyz=(0.435, -0.417, 0.0)), material=powder, name="duct_sleeve_side_0")
    housing.visual(Box((0.155, 0.026, 0.280)), origin=Origin(xyz=(0.435, 0.417, 0.0)), material=powder, name="duct_sleeve_side_1")
    housing.visual(Box((0.024, 0.99, 0.060)), origin=Origin(xyz=(0.508, 0.0, 0.165)), material=powder, name="mount_flange_top")
    housing.visual(Box((0.024, 0.99, 0.060)), origin=Origin(xyz=(0.508, 0.0, -0.165)), material=powder, name="mount_flange_bottom")
    housing.visual(Box((0.024, 0.035, 0.390)), origin=Origin(xyz=(0.508, -0.4775, 0.0)), material=powder, name="mount_flange_side_0")
    housing.visual(Box((0.024, 0.035, 0.390)), origin=Origin(xyz=(0.508, 0.4775, 0.0)), material=powder, name="mount_flange_side_1")
    # Rear service opening trim and slide-guide ledges.
    housing.visual(Box((0.026, 1.20, 0.050)), origin=Origin(xyz=(-0.362, 0.0, 0.200)), material=powder, name="rear_lip_top")
    housing.visual(Box((0.026, 1.20, 0.050)), origin=Origin(xyz=(-0.362, 0.0, -0.200)), material=powder, name="rear_lip_bottom")
    housing.visual(Box((0.026, 0.110, 0.360)), origin=Origin(xyz=(-0.362, -0.545, 0.0)), material=powder, name="rear_lip_side_0")
    housing.visual(Box((0.026, 0.110, 0.360)), origin=Origin(xyz=(-0.362, 0.545, 0.0)), material=powder, name="rear_lip_side_1")
    housing.visual(Box((0.430, 0.110, 0.022)), origin=Origin(xyz=(-0.125, -0.500, -0.153)), material=powder, name="lower_guide_0")
    housing.visual(Box((0.430, 0.110, 0.022)), origin=Origin(xyz=(-0.125, 0.500, -0.153)), material=powder, name="lower_guide_1")
    housing.visual(Box((0.430, 0.110, 0.018)), origin=Origin(xyz=(-0.125, -0.500, 0.155)), material=powder, name="upper_guide_0")
    housing.visual(Box((0.430, 0.110, 0.018)), origin=Origin(xyz=(-0.125, 0.500, 0.155)), material=powder, name="upper_guide_1")
    # Dark seams and front gasket strips are seated into exterior metal faces.
    housing.visual(Box((0.620, 0.012, 0.010)), origin=Origin(xyz=(-0.020, 0.0, 0.235)), material=dark, name="top_split_seam")
    housing.visual(Box((0.012, 0.92, 0.014)), origin=Origin(xyz=(0.520, 0.0, 0.188)), material=dark, name="front_top_gasket")
    housing.visual(Box((0.012, 0.92, 0.014)), origin=Origin(xyz=(0.520, 0.0, -0.188)), material=dark, name="front_bottom_gasket")
    housing.visual(
        Box((0.050, 0.050, 0.030)),
        origin=Origin(xyz=(-0.370, -0.465, 0.205)),
        material=hinge_metal,
        name="hinge_base_0",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.13),
        origin=Origin(xyz=(-0.397, -0.40, 0.205), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=hinge_metal,
        name="fixed_pin_boss_0",
    )
    housing.visual(
        Box((0.050, 0.050, 0.030)),
        origin=Origin(xyz=(-0.370, 0.465, 0.205)),
        material=hinge_metal,
        name="hinge_base_1",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.13),
        origin=Origin(xyz=(-0.397, 0.40, 0.205), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=hinge_metal,
        name="fixed_pin_boss_1",
    )

    door = model.part("rear_door")
    door.visual(Box((0.022, 1.00, 0.335)), origin=Origin(xyz=(0.0, 0.0, -0.185)), material=powder, name="door_panel")
    door.visual(Box((0.012, 0.94, 0.018)), origin=Origin(xyz=(0.017, 0.0, -0.030)), material=dark, name="top_gasket")
    door.visual(Box((0.012, 0.94, 0.018)), origin=Origin(xyz=(0.017, 0.0, -0.335)), material=dark, name="bottom_gasket")
    door.visual(Box((0.012, 0.018, 0.320)), origin=Origin(xyz=(0.017, -0.470, -0.185)), material=dark, name="gasket_0")
    door.visual(Box((0.012, 0.018, 0.320)), origin=Origin(xyz=(0.017, 0.470, -0.185)), material=dark, name="gasket_1")
    door.visual(Box((0.026, 0.48, 0.036)), origin=Origin(xyz=(0.002, 0.0, -0.012)), material=hinge_metal, name="hinge_leaf")
    door.visual(
        Cylinder(radius=0.020, length=0.145),
        origin=Origin(xyz=(-0.002, -0.34, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=hinge_metal,
        name="hinge_knuckle_0",
    )
    door.visual(
        Cylinder(radius=0.020, length=0.145),
        origin=Origin(xyz=(-0.002, 0.34, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=hinge_metal,
        name="hinge_knuckle_1",
    )
    # A raised pull handle on the rear face.
    door.visual(Box((0.052, 0.030, 0.070)), origin=Origin(xyz=(-0.037, -0.155, -0.235)), material=dark, name="handle_post_0")
    door.visual(Box((0.052, 0.030, 0.070)), origin=Origin(xyz=(-0.037, 0.155, -0.235)), material=dark, name="handle_post_1")
    door.visual(Box((0.034, 0.340, 0.030)), origin=Origin(xyz=(-0.072, 0.0, -0.235)), material=dark, name="pull_handle")

    filter_frame = model.part("filter_frame")
    filter_frame.visual(Box((0.310, 0.800, 0.235)), origin=Origin(xyz=(0.175, 0.0, 0.0)), material=filter_media, name="media_block")
    filter_frame.visual(Box((0.340, 0.900, 0.030)), origin=Origin(xyz=(0.175, 0.0, 0.127)), material=blue_plastic, name="top_rail")
    filter_frame.visual(Box((0.340, 0.900, 0.030)), origin=Origin(xyz=(0.175, 0.0, -0.127)), material=blue_plastic, name="bottom_rail")
    filter_frame.visual(Box((0.340, 0.030, 0.300)), origin=Origin(xyz=(0.175, -0.415, 0.0)), material=blue_plastic, name="side_rail_0")
    filter_frame.visual(Box((0.340, 0.030, 0.300)), origin=Origin(xyz=(0.175, 0.415, 0.0)), material=blue_plastic, name="side_rail_1")
    filter_frame.visual(Box((0.032, 0.270, 0.075)), origin=Origin(xyz=(-0.004, 0.0, -0.010)), material=blue_plastic, name="pull_tab")
    filter_frame.visual(Box((0.020, 0.790, 0.018)), origin=Origin(xyz=(0.018, 0.0, 0.0)), material=blue_plastic, name="rear_crossbar")
    for idx, y in enumerate((-0.315, -0.210, -0.105, 0.0, 0.105, 0.210, 0.315)):
        filter_frame.visual(
            Box((0.012, 0.018, 0.218)),
            origin=Origin(xyz=(0.014, y, 0.0)),
            material=pale_blue,
            name=f"pleat_{idx}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-0.392, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "filter_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_frame,
        origin=Origin(xyz=(-0.335, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.285),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("rear_door")
    filter_frame = object_model.get_part("filter_frame")
    door_hinge = object_model.get_articulation("door_hinge")
    filter_slide = object_model.get_articulation("filter_slide")

    # These are local seated details on the same manufactured casing or hinge
    # representation rather than unintended part-to-part collisions.
    ctx.allow_overlap(
        "housing",
        "rear_door",
        elem_a="fixed_pin_boss_0",
        elem_b="hinge_knuckle_0",
        reason="The visible hinge pin/boss is intentionally captured inside the door hinge knuckle.",
    )
    ctx.allow_overlap(
        "housing",
        "rear_door",
        elem_a="fixed_pin_boss_1",
        elem_b="hinge_knuckle_1",
        reason="The visible hinge pin/boss is intentionally captured inside the door hinge knuckle.",
    )
    ctx.allow_overlap(
        "housing",
        "rear_door",
        elem_a="rear_lip_top",
        elem_b="top_gasket",
        reason="The service-door gasket is intentionally compressed against the rear opening lip when latched.",
    )
    ctx.expect_contact(
        housing,
        door,
        elem_a="fixed_pin_boss_0",
        elem_b="hinge_knuckle_0",
        contact_tol=0.020,
        name="first hinge knuckle is mounted on its pin boss",
    )
    ctx.expect_contact(
        housing,
        door,
        elem_a="fixed_pin_boss_1",
        elem_b="hinge_knuckle_1",
        contact_tol=0.020,
        name="second hinge knuckle is mounted on its pin boss",
    )
    ctx.expect_gap(
        housing,
        door,
        axis="z",
        positive_elem="rear_lip_top",
        negative_elem="top_gasket",
        max_penetration=0.010,
        name="top door gasket is locally compressed against rear lip",
    )

    ctx.expect_within(
        filter_frame,
        housing,
        axes="yz",
        inner_elem="media_block",
        margin=0.010,
        name="filter media fits inside rear service opening",
    )
    ctx.expect_overlap(
        filter_frame,
        housing,
        axes="x",
        elem_a="media_block",
        min_overlap=0.18,
        name="filter cassette is deeply retained when inserted",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.expect_gap(
            filter_frame,
            door,
            axis="x",
            min_gap=0.005,
            positive_elem="pull_tab",
            negative_elem="door_panel",
            name="opened door clears the filter pull tab",
        )

    ctx.check(
        "door hinge swings rear hatch outward and upward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][0] < closed_door_aabb[0][0] - 0.10
        and open_door_aabb[0][2] > closed_door_aabb[0][2] + 0.05,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_filter = ctx.part_world_position(filter_frame)
    with ctx.pose({door_hinge: 1.35, filter_slide: 0.285}):
        extended_filter = ctx.part_world_position(filter_frame)
        ctx.expect_overlap(
            filter_frame,
            housing,
            axes="x",
            elem_a="media_block",
            min_overlap=0.025,
            name="extended filter remains captured in guide rails",
        )
        ctx.expect_within(
            filter_frame,
            housing,
            axes="yz",
            inner_elem="media_block",
            margin=0.010,
            name="extended filter stays centered in the housing aperture",
        )
    ctx.check(
        "filter slide moves out through rear opening",
        rest_filter is not None and extended_filter is not None and extended_filter[0] < rest_filter[0] - 0.20,
        details=f"rest={rest_filter}, extended={extended_filter}",
    )

    return ctx.report()


object_model = build_object_model()
