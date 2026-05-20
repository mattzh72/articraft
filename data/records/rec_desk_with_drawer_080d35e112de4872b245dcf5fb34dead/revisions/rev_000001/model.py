from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


FRONT_Y = -0.365
PEDESTAL_DEPTH = 0.62
PEDESTAL_FRONT = -0.335
PEDESTAL_REAR = PEDESTAL_FRONT + PEDESTAL_DEPTH
PEDESTAL_WIDTH = 0.48
PANEL = 0.035
INNER_HALF = (PEDESTAL_WIDTH - 2.0 * PANEL) / 2.0

FACE_THICKNESS = 0.035
SLIDE_RAIL_H = 0.018
FIXED_RAIL_H = 0.014


def _add_box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _make_oval_meshes():
    outer = superellipse_profile(0.165, 0.062, exponent=2.15, segments=64)
    inner = superellipse_profile(0.126, 0.035, exponent=2.05, segments=64)
    ring = mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [inner], 0.006, cap=True, center=True),
        "recessed_oval_pull_ring",
    )
    cup = mesh_from_geometry(
        ExtrudeGeometry(
            superellipse_profile(0.126, 0.035, exponent=2.05, segments=64),
            0.004,
            cap=True,
            center=True,
        ),
        "recessed_oval_pull_cup",
    )
    return ring, cup


def _add_drawer(
    model,
    desk,
    *,
    name,
    pedestal_name,
    pedestal_x,
    center_z,
    face_h,
    body_depth,
    travel,
    wood,
    interior_wood,
    brass,
    shadow,
    steel,
    handle_ring,
    handle_cup,
):
    drawer = model.part(name)

    face_w = 0.405
    body_w = 0.340
    body_h = max(0.075, face_h - 0.045)
    body_front = FACE_THICKNESS / 2.0 - 0.003
    body_y = body_front + body_depth / 2.0
    body_rear_y = body_front + body_depth
    floor_h = 0.018
    wall_t = 0.018

    _add_box(drawer, (face_w, FACE_THICKNESS, face_h), (0.0, 0.0, 0.0), wood, "front")
    # Subtle raised perimeter rails make each drawer front read as a separate
    # framed wood face rather than a plain box.
    _add_box(drawer, (face_w - 0.030, 0.010, 0.012), (0.0, -0.020, face_h / 2.0 - 0.020), wood, "front_top_lip")
    _add_box(drawer, (face_w - 0.030, 0.010, 0.012), (0.0, -0.020, -face_h / 2.0 + 0.020), wood, "front_bottom_lip")
    _add_box(drawer, (0.012, 0.010, face_h - 0.050), (-face_w / 2.0 + 0.025, -0.020, 0.0), wood, "front_side_lip_0")
    _add_box(drawer, (0.012, 0.010, face_h - 0.050), (face_w / 2.0 - 0.025, -0.020, 0.0), wood, "front_side_lip_1")

    # Open wooden drawer box behind the face: floor, side walls, and rear wall.
    _add_box(
        drawer,
        (body_w, body_depth, floor_h),
        (0.0, body_y, -body_h / 2.0 + floor_h / 2.0),
        interior_wood,
        "floor",
    )
    _add_box(
        drawer,
        (wall_t, body_depth, body_h),
        (-body_w / 2.0 + wall_t / 2.0, body_y, 0.0),
        interior_wood,
        "side_0",
    )
    _add_box(
        drawer,
        (wall_t, body_depth, body_h),
        (body_w / 2.0 - wall_t / 2.0, body_y, 0.0),
        interior_wood,
        "side_1",
    )
    _add_box(
        drawer,
        (body_w, wall_t, body_h),
        (0.0, body_rear_y - wall_t / 2.0, 0.0),
        interior_wood,
        "rear")

    # Moving half of the side guide rails, attached to the drawer box.
    rail_depth = min(0.490, body_depth - 0.040)
    rail_y = body_front + 0.035 + rail_depth / 2.0
    rail_z = -body_h / 2.0 + 0.032
    for idx, sign in enumerate((-1.0, 1.0)):
        _add_box(
            drawer,
            (0.020, rail_depth, SLIDE_RAIL_H),
            (sign * (body_w / 2.0 + 0.010), rail_y, rail_z),
            steel,
            f"slide_rail_{idx}",
        )

    # Recessed oval pull: brass rim with a dark inset cup flush into the face.
    drawer.visual(
        handle_cup,
        origin=Origin(xyz=(0.0, -FACE_THICKNESS / 2.0 - 0.0005, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="handle_cup",
    )
    drawer.visual(
        handle_ring,
        origin=Origin(xyz=(0.0, -FACE_THICKNESS / 2.0 - 0.0020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="handle_rim",
    )

    model.articulation(
        f"desk_to_{name}",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin(xyz=(pedestal_x, FRONT_Y, center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.32, lower=0.0, upper=travel),
    )

    fixed_rail_z = center_z + rail_z - (SLIDE_RAIL_H + FIXED_RAIL_H) / 2.0
    fixed_y = PEDESTAL_FRONT + 0.050 + rail_depth / 2.0
    for idx, sign in enumerate((-1.0, 1.0)):
        _add_box(
            desk,
            (0.025, rail_depth, FIXED_RAIL_H),
            (pedestal_x + sign * (INNER_HALF - 0.0125), fixed_y, fixed_rail_z),
            steel,
            f"{pedestal_name}_fixed_rail_{name[-1]}_{idx}",
        )

    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_writing_desk")

    wood = model.material("warm_walnut", rgba=(0.43, 0.24, 0.11, 1.0))
    endgrain = model.material("dark_walnut_endgrain", rgba=(0.27, 0.13, 0.055, 1.0))
    interior_wood = model.material("drawer_interior_maple", rgba=(0.62, 0.46, 0.28, 1.0))
    brass = model.material("aged_brass", rgba=(0.82, 0.61, 0.25, 1.0))
    shadow = model.material("dark_recess_shadow", rgba=(0.025, 0.020, 0.016, 1.0))
    steel = model.material("dull_steel_rails", rgba=(0.46, 0.47, 0.46, 1.0))

    handle_ring, handle_cup = _make_oval_meshes()

    desk = model.part("desk_frame")

    # Wide writing top with a slight overhang and darker edge banding.
    _add_box(desk, (1.62, 0.76, 0.050), (0.0, 0.0, 0.735), wood, "wide_rectangular_top")
    _add_box(desk, (1.66, 0.030, 0.065), (0.0, -0.395, 0.725), endgrain, "front_top_band")
    _add_box(desk, (1.66, 0.024, 0.055), (0.0, 0.392, 0.725), endgrain, "rear_top_band")
    _add_box(desk, (0.030, 0.76, 0.055), (-0.825, 0.0, 0.725), endgrain, "top_end_band_0")
    _add_box(desk, (0.030, 0.76, 0.055), (0.825, 0.0, 0.725), endgrain, "top_end_band_1")

    # Two connected pedestal carcasses under the top, plus a rear modesty panel
    # that visually ties the desk into one rigid writing-desk frame.
    pedestal_centers = {"left": -0.51, "right": 0.51}
    for ped_name, x in pedestal_centers.items():
        _add_box(desk, (PANEL, PEDESTAL_DEPTH, 0.690), (x - PEDESTAL_WIDTH / 2.0 + PANEL / 2.0, -0.025, 0.365), wood, f"{ped_name}_outer_side")
        _add_box(desk, (PANEL, PEDESTAL_DEPTH, 0.690), (x + PEDESTAL_WIDTH / 2.0 - PANEL / 2.0, -0.025, 0.365), wood, f"{ped_name}_inner_side")
        _add_box(desk, (PEDESTAL_WIDTH, PEDESTAL_DEPTH, PANEL), (x, -0.025, 0.0175), endgrain, f"{ped_name}_base_panel")
        _add_box(desk, (PEDESTAL_WIDTH, 0.050, 0.070), (x, PEDESTAL_REAR - 0.025, 0.365), wood, f"{ped_name}_rear_stile")
        _add_box(desk, (PEDESTAL_WIDTH, 0.030, 0.045), (x, PEDESTAL_FRONT + 0.022, 0.690), endgrain, f"{ped_name}_top_front_rail")
        _add_box(desk, (PEDESTAL_WIDTH, 0.030, 0.040), (x, PEDESTAL_FRONT + 0.022, 0.053), endgrain, f"{ped_name}_bottom_front_rail")
        _add_box(desk, (0.075, 0.050, 0.060), (x - 0.155, PEDESTAL_FRONT - 0.025, 0.030), endgrain, f"{ped_name}_front_foot_0")
        _add_box(desk, (0.075, 0.050, 0.060), (x + 0.155, PEDESTAL_FRONT - 0.025, 0.030), endgrain, f"{ped_name}_front_foot_1")

    _add_box(desk, (0.58, 0.035, 0.400), (0.0, PEDESTAL_REAR - 0.020, 0.390), wood, "rear_modesty_panel")
    _add_box(desk, (0.58, 0.040, 0.055), (0.0, PEDESTAL_FRONT - 0.010, 0.690), endgrain, "kneehole_front_rail")

    # Face-frame divider rails defining three equal left drawers and three
    # graduated right drawers while leaving clear openings for the drawer boxes.
    for i, z in enumerate((0.265, 0.475)):
        _add_box(desk, (PEDESTAL_WIDTH, 0.030, 0.025), (pedestal_centers["left"], PEDESTAL_FRONT + 0.022, z), endgrain, f"left_divider_rail_{i}")
    for i, z in enumerate((0.3325, 0.5475)):
        _add_box(desk, (PEDESTAL_WIDTH, 0.030, 0.025), (pedestal_centers["right"], PEDESTAL_FRONT + 0.022, z), endgrain, f"right_divider_rail_{i}")

    for idx, z in enumerate((0.160, 0.370, 0.580)):
        _add_drawer(
            model,
            desk,
            name=f"left_drawer_{idx}",
            pedestal_name="left",
            pedestal_x=pedestal_centers["left"],
            center_z=z,
            face_h=0.180,
            body_depth=0.500,
            travel=0.320,
            wood=wood,
            interior_wood=interior_wood,
            brass=brass,
            shadow=shadow,
            steel=steel,
            handle_ring=handle_ring,
            handle_cup=handle_cup,
        )

    right_specs = (
        (0, 0.620, 0.130, 0.430, 0.260),
        (1, 0.440, 0.190, 0.480, 0.310),
        (2, 0.195, 0.250, 0.550, 0.360),
    )
    for idx, z, face_h, body_depth, travel in right_specs:
        _add_drawer(
            model,
            desk,
            name=f"right_drawer_{idx}",
            pedestal_name="right",
            pedestal_x=pedestal_centers["right"],
            center_z=z,
            face_h=face_h,
            body_depth=body_depth,
            travel=travel,
            wood=wood,
            interior_wood=interior_wood,
            brass=brass,
            shadow=shadow,
            steel=steel,
            handle_ring=handle_ring,
            handle_cup=handle_cup,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk = object_model.get_part("desk_frame")

    for drawer_name in (
        "left_drawer_0",
        "left_drawer_1",
        "left_drawer_2",
        "right_drawer_0",
        "right_drawer_1",
        "right_drawer_2",
    ):
        drawer = object_model.get_part(drawer_name)
        joint = object_model.get_articulation(f"desk_to_{drawer_name}")
        side = "left" if drawer_name.startswith("left") else "right"
        idx = drawer_name[-1]

        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="slide_rail_0",
            elem_b=f"{side}_fixed_rail_{idx}_0",
            min_overlap=0.18,
            name=f"{drawer_name} rail retains insertion",
        )
        ctx.expect_gap(
            drawer,
            desk,
            axis="z",
            positive_elem="slide_rail_0",
            negative_elem=f"{side}_fixed_rail_{idx}_0",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{drawer_name} rail sits on guide",
        )

        rest = ctx.part_world_position(drawer)
        upper = joint.motion_limits.upper
        with ctx.pose({joint: upper}):
            ctx.expect_overlap(
                drawer,
                desk,
                axes="y",
                elem_a="slide_rail_0",
                elem_b=f"{side}_fixed_rail_{idx}_0",
                min_overlap=0.09,
                name=f"{drawer_name} rail remains engaged when extended",
            )
            extended = ctx.part_world_position(drawer)
        ctx.check(
            f"{drawer_name} pulls toward the user",
            rest is not None and extended is not None and extended[1] < rest[1] - 0.10,
            details=f"rest={rest}, extended={extended}",
        )

    def front_height(part_name: str) -> float:
        aabb = ctx.part_element_world_aabb(object_model.get_part(part_name), elem="front")
        if aabb is None:
            return 0.0
        lo, hi = aabb
        return hi[2] - lo[2]

    h_top = front_height("right_drawer_0")
    h_mid = front_height("right_drawer_1")
    h_bottom = front_height("right_drawer_2")
    ctx.check(
        "right pedestal drawers are graduated",
        h_top < h_mid < h_bottom,
        details=f"right drawer front heights: top={h_top}, middle={h_mid}, bottom={h_bottom}",
    )

    return ctx.report()


object_model = build_object_model()
