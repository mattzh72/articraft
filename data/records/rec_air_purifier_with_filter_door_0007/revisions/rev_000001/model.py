from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_RADIUS = 0.09
BODY_HEIGHT = 0.29
BODY_INNER_RADIUS = 0.078


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_body_shell_mesh() -> MeshGeometry:
    outer = CylinderGeometry(radius=BODY_RADIUS, height=BODY_HEIGHT, radial_segments=72)
    inner = CylinderGeometry(
        radius=BODY_INNER_RADIUS,
        height=BODY_HEIGHT - 0.040,
        radial_segments=72,
    )
    shell = boolean_difference(outer, inner)

    front_opening = BoxGeometry((0.058, 0.118, 0.186)).translate(0.061, 0.0, 0.0)
    shell = boolean_difference(shell, front_opening)

    outlet_recess = CylinderGeometry(radius=0.070, height=0.020, radial_segments=64).translate(
        0.0,
        0.0,
        (BODY_HEIGHT * 0.5) - 0.010,
    )
    shell = boolean_difference(shell, outlet_recess)
    return shell.translate(0.0, 0.0, BODY_HEIGHT * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_air_purifier", assets=ASSETS)

    shell_mat = model.material("shell_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    door_mat = model.material("door_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    dark_mat = model.material("dark_trim", rgba=(0.09, 0.10, 0.11, 1.0))
    filter_mat = model.material("filter_media", rgba=(0.33, 0.36, 0.38, 1.0))
    steel_mat = model.material("hardware", rgba=(0.54, 0.56, 0.58, 1.0))
    satin_mat = model.material("satin_trim", rgba=(0.31, 0.33, 0.35, 1.0))

    opening_w = 0.102
    opening_h = 0.158
    lip_t = 0.008
    lip_x = 0.078
    lip_side_w = 0.010
    lip_rail_h = 0.012

    bay_side_w = 0.010
    bay_rail_h = 0.012
    bay_frame_x = 0.065
    bay_back_x = 0.053

    door_w = 0.114
    door_h = 0.172
    door_t = 0.0045
    hinge_x = BODY_RADIUS + 0.0025
    hinge_y = door_w * 0.5
    hinge_z = 0.145
    hinge_pin_r = 0.0016

    body = model.part("body")
    body.visual(
        _save_mesh(_build_body_shell_mesh(), "purifier_shell.obj"),
        material=shell_mat,
        name="shell",
    )
    body.visual(
        Cylinder(radius=0.095, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_mat,
        name="base_ring",
    )

    seat_side_offset = (door_w * 0.5) - 0.005
    seat_rail_offset = (door_h * 0.5) - 0.005
    seat_x = hinge_x - door_t - 0.001
    body.visual(
        Box((0.002, 0.010, door_h - 0.004)),
        origin=Origin(xyz=(seat_x, -seat_side_offset, hinge_z)),
        material=door_mat,
        name="seat_left",
    )
    body.visual(
        Box((0.002, 0.010, door_h - 0.004)),
        origin=Origin(xyz=(seat_x, seat_side_offset, hinge_z)),
        material=door_mat,
        name="seat_right",
    )
    body.visual(
        Box((0.002, door_w - 0.020, 0.010)),
        origin=Origin(xyz=(seat_x, 0.0, hinge_z + seat_rail_offset)),
        material=door_mat,
        name="seat_top",
    )
    body.visual(
        Box((0.002, door_w - 0.020, 0.010)),
        origin=Origin(xyz=(seat_x, 0.0, hinge_z - seat_rail_offset)),
        material=door_mat,
        name="seat_bottom",
    )

    side_offset = (opening_w * 0.5) + (lip_side_w * 0.5)
    rail_offset = (opening_h * 0.5) + (lip_rail_h * 0.5)
    body.visual(
        Box((lip_t, lip_side_w, opening_h + (lip_rail_h * 2.0))),
        origin=Origin(xyz=(lip_x, -side_offset, hinge_z)),
        material=shell_mat,
        name="lip_left",
    )
    body.visual(
        Box((lip_t, lip_side_w, opening_h + (lip_rail_h * 2.0))),
        origin=Origin(xyz=(lip_x, side_offset, hinge_z)),
        material=satin_mat,
        name="lip_right",
    )
    body.visual(
        Box((lip_t, opening_w, lip_rail_h)),
        origin=Origin(xyz=(lip_x, 0.0, hinge_z + rail_offset)),
        material=satin_mat,
        name="lip_top",
    )
    body.visual(
        Box((lip_t, opening_w, lip_rail_h)),
        origin=Origin(xyz=(lip_x, 0.0, hinge_z - rail_offset)),
        material=satin_mat,
        name="lip_bottom",
    )

    bay_side_offset = (opening_w * 0.5) - (bay_side_w * 0.5)
    bay_rail_offset = (opening_h * 0.5) - (bay_rail_h * 0.5)
    body.visual(
        Box((0.020, bay_side_w, opening_h - 0.004)),
        origin=Origin(xyz=(bay_frame_x, -bay_side_offset, hinge_z)),
        material=shell_mat,
        name="bay_left_frame",
    )
    body.visual(
        Box((0.020, bay_side_w, opening_h - 0.004)),
        origin=Origin(xyz=(bay_frame_x, bay_side_offset, hinge_z)),
        material=shell_mat,
        name="bay_right_frame",
    )
    body.visual(
        Box((0.020, opening_w - 0.020, bay_rail_h)),
        origin=Origin(xyz=(bay_frame_x, 0.0, hinge_z + bay_rail_offset)),
        material=shell_mat,
        name="bay_top_frame",
    )
    body.visual(
        Box((0.020, opening_w - 0.020, bay_rail_h)),
        origin=Origin(xyz=(bay_frame_x, 0.0, hinge_z - bay_rail_offset)),
        material=shell_mat,
        name="bay_bottom_frame",
    )
    body.visual(
        Box((0.006, opening_w - 0.014, opening_h - 0.010)),
        origin=Origin(xyz=(bay_back_x, 0.0, hinge_z)),
        material=dark_mat,
        name="bay_backplate",
    )
    body.visual(
        Box((0.012, 0.086, 0.140)),
        origin=Origin(xyz=(0.057, 0.0, hinge_z)),
        material=filter_mat,
        name="bay_filter",
    )
    for idx, y in enumerate((-0.028, -0.014, 0.0, 0.014, 0.028), start=1):
        body.visual(
            Box((0.014, 0.004, 0.140)),
            origin=Origin(xyz=(0.059, y, hinge_z)),
            material=dark_mat,
            name=f"filter_rib_{idx}",
        )

    for prefix, z in (("upper", hinge_z + 0.064), ("lower", hinge_z - 0.064)):
        body.visual(
            Box((0.011, 0.010, 0.030)),
            origin=Origin(xyz=(0.0855, hinge_y - 0.001, z)),
            material=shell_mat,
            name=f"{prefix}_hinge_mount",
        )
        body.visual(
            Cylinder(radius=hinge_pin_r, length=0.022),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material=steel_mat,
            name=f"{prefix}_hinge_pin",
        )
    body.visual(
        Cylinder(radius=hinge_pin_r, length=0.128),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        material=steel_mat,
        name="hinge_spindle",
    )

    body.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.021)),
        material=dark_mat,
        name="outlet_floor",
    )
    for idx, y in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030), start=1):
        body.visual(
            Box((0.112, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, y, BODY_HEIGHT - 0.013)),
            material=shell_mat,
            name="grille_slat_center" if idx == 3 else f"grille_slat_{idx}",
        )
    body.visual(
        Box((0.050, 0.018, 0.003)),
        origin=Origin(xyz=(0.022, 0.0, BODY_HEIGHT - 0.023)),
        material=dark_mat,
        name="control_strip",
    )
    body.visual(
        Box((0.026, 0.005, 0.0015)),
        origin=Origin(xyz=(0.022, 0.0, BODY_HEIGHT - 0.021)),
        material=steel_mat,
        name="control_highlight",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS, length=BODY_HEIGHT),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    door = model.part("filter_door")
    door.visual(
        Box((door_t, door_w, door_h)),
        origin=Origin(xyz=(-door_t * 0.5, -door_w * 0.5, 0.0)),
        material=door_mat,
        name="door_panel",
    )
    door.visual(
        Box((0.0018, 0.088, 0.142)),
        origin=Origin(xyz=(-0.0016, -door_w * 0.5, 0.0)),
        material=satin_mat,
        name="door_inner_stiffener",
    )
    door.visual(
        Box((0.003, 0.010, door_h - 0.004)),
        origin=Origin(xyz=(-0.0015, -door_w + 0.005, 0.0)),
        material=door_mat,
        name="free_edge_return",
    )
    for name, z in (("upper_knuckle", 0.064), ("lower_knuckle", -0.064)):
        door.visual(
            Cylinder(radius=0.0038, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=shell_mat,
            name=name,
        )
    pocket_y = -door_w + 0.008
    door.visual(
        Box((0.0032, 0.015, 0.060)),
        origin=Origin(xyz=(-0.0016, pocket_y, 0.0)),
        material=dark_mat,
        name="pull_pocket",
    )
    door.visual(
        Cylinder(radius=0.0044, length=0.040),
        origin=Origin(xyz=(-0.0003, pocket_y, 0.0)),
        material=steel_mat,
        name="pull_core",
    )
    for idx, z in enumerate((-0.014, -0.007, 0.0, 0.007, 0.014), start=1):
        door.visual(
            Cylinder(radius=0.0049, length=0.003),
            origin=Origin(xyz=(-0.0003, pocket_y, z)),
            material=steel_mat,
            name=f"knurl_band_{idx}",
        )
    door.inertial = Inertial.from_geometry(
        Box((0.018, door_w, door_h)),
        mass=0.45,
        origin=Origin(xyz=(-0.009, -door_w * 0.5, 0.0)),
    )

    model.articulation(
        "filter_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.2, lower=0.0, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    door = object_model.get_part("filter_door")
    hinge = object_model.get_articulation("filter_door_hinge")

    seat_top = body.get_visual("seat_top")
    lip_top = body.get_visual("lip_top")
    bay_filter = body.get_visual("bay_filter")
    bay_left_frame = body.get_visual("bay_left_frame")
    bay_right_frame = body.get_visual("bay_right_frame")
    bay_top_frame = body.get_visual("bay_top_frame")
    bay_bottom_frame = body.get_visual("bay_bottom_frame")
    outlet_floor = body.get_visual("outlet_floor")
    upper_hinge_pin = body.get_visual("upper_hinge_pin")
    lower_hinge_pin = body.get_visual("lower_hinge_pin")
    grille_slat_center = body.get_visual("grille_slat_center")
    control_strip = body.get_visual("control_strip")

    door_panel = door.get_visual("door_panel")
    upper_knuckle = door.get_visual("upper_knuckle")
    lower_knuckle = door.get_visual("lower_knuckle")
    pull_pocket = door.get_visual("pull_pocket")
    pull_core = door.get_visual("pull_core")
    free_edge_return = door.get_visual("free_edge_return")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(body, door, reason="flush knuckle barrels sleeve over fixed hinge pins at the purifier door edge")

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.004)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        door,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=door_panel,
        negative_elem=seat_top,
        name="door_panel_seats_on_front_frame",
    )
    ctx.expect_gap(
        door,
        body,
        axis="x",
        min_gap=0.003,
        max_gap=0.012,
        positive_elem=door_panel,
        negative_elem=lip_top,
        name="molded_lip_set_back_behind_closed_door",
    )
    ctx.expect_gap(
        body,
        body,
        axis="x",
        min_gap=0.010,
        max_gap=0.030,
        positive_elem=lip_top,
        negative_elem=bay_filter,
        name="filter_bay_sits_behind_molded_lip",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="yz",
        elem_a=bay_filter,
        elem_b=bay_left_frame,
        min_overlap=0.001,
        name="filter_media_captured_by_left_frame",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="yz",
        elem_a=bay_filter,
        elem_b=bay_right_frame,
        min_overlap=0.001,
        name="filter_media_captured_by_right_frame",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="yz",
        elem_a=bay_filter,
        elem_b=bay_top_frame,
        min_overlap=0.001,
        name="filter_media_captured_by_top_frame",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="yz",
        elem_a=bay_filter,
        elem_b=bay_bottom_frame,
        min_overlap=0.001,
        name="filter_media_captured_by_bottom_frame",
    )
    ctx.expect_within(body, door, axes="xy", inner_elem=upper_hinge_pin, outer_elem=upper_knuckle)
    ctx.expect_within(body, door, axes="xy", inner_elem=lower_hinge_pin, outer_elem=lower_knuckle)
    ctx.expect_within(door, door, axes="yz", inner_elem=pull_core, outer_elem=pull_pocket)
    ctx.expect_overlap(
        door,
        door,
        axes="yz",
        elem_a=pull_pocket,
        elem_b=free_edge_return,
        min_overlap=0.005,
        name="pull_tab_set_into_free_edge",
    )
    ctx.expect_within(body, body, axes="xy", inner_elem=grille_slat_center, outer_elem=outlet_floor)
    ctx.expect_within(body, body, axes="xy", inner_elem=control_strip, outer_elem=outlet_floor)
    ctx.expect_gap(
        body,
        body,
        axis="z",
        min_gap=0.006,
        max_gap=0.014,
        positive_elem=grille_slat_center,
        negative_elem=control_strip,
        name="control_strip_recessed_below_grille",
    )

    with ctx.pose({hinge: 1.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            min_gap=0.06,
            positive_elem=free_edge_return,
            negative_elem=bay_filter,
            name="door_swings_clear_to_expose_filter_bay",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
