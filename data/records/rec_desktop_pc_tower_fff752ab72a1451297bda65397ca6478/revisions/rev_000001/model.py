from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_beige_desktop_tower")

    model.material("aged_beige", rgba=(0.72, 0.66, 0.53, 1.0))
    model.material("slightly_darker_beige", rgba=(0.58, 0.53, 0.43, 1.0))
    model.material("dark_drive_plastic", rgba=(0.035, 0.034, 0.032, 1.0))
    model.material("warm_shadow", rgba=(0.10, 0.09, 0.075, 1.0))
    model.material("brushed_metal", rgba=(0.54, 0.55, 0.53, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.015, 0.014, 1.0))
    model.material("green_led", rgba=(0.02, 0.80, 0.18, 1.0))
    model.material("amber_led", rgba=(1.0, 0.55, 0.08, 1.0))

    chassis = model.part("chassis")

    # Coordinate frame: +X is the front of the tower, +Z is up, and +Y is the
    # hinged service side.  The case is full-size for a beige AT/ATX mini tower.
    case_depth = 0.420
    case_width = 0.200
    case_height = 0.460
    side_y = case_width / 2.0
    rear_x = -case_depth / 2.0
    front_x = case_depth / 2.0
    sheet = 0.012

    # Sheet-metal/plastic enclosure, built as connected panels with the service
    # side left open for the articulated side cover.
    chassis.visual(
        Box((case_depth, sheet, case_height)),
        origin=Origin(xyz=(0.0, -side_y + sheet / 2.0, case_height / 2.0)),
        material="aged_beige",
        name="fixed_side",
    )
    chassis.visual(
        Box((case_depth, case_width, sheet)),
        origin=Origin(xyz=(0.0, 0.0, case_height - sheet / 2.0)),
        material="aged_beige",
        name="top_panel",
    )
    chassis.visual(
        Box((case_depth, case_width, sheet)),
        origin=Origin(xyz=(0.0, 0.0, sheet / 2.0)),
        material="aged_beige",
        name="bottom_panel",
    )
    chassis.visual(
        Box((sheet, case_width, case_height)),
        origin=Origin(xyz=(rear_x + sheet / 2.0, 0.0, case_height / 2.0)),
        material="aged_beige",
        name="rear_panel",
    )

    # Front fascia is segmented around a real 5.25-inch bay aperture instead of
    # being one solid slab.  The bay opening is slightly larger than a 146 mm
    # drive face, with visible margins and a proud molded trim ring.
    bay_width = 0.154
    bay_height = 0.052
    bay_center_z = 0.360
    bay_min_z = bay_center_z - bay_height / 2.0
    bay_max_z = bay_center_z + bay_height / 2.0
    front_t = 0.018
    front_center_x = front_x - front_t / 2.0
    stile_w = (case_width - bay_width) / 2.0
    chassis.visual(
        Box((front_t, stile_w, case_height)),
        origin=Origin(xyz=(front_center_x, side_y - stile_w / 2.0, case_height / 2.0)),
        material="aged_beige",
        name="front_stile_0",
    )
    chassis.visual(
        Box((front_t, stile_w, case_height)),
        origin=Origin(xyz=(front_center_x, -side_y + stile_w / 2.0, case_height / 2.0)),
        material="aged_beige",
        name="front_stile_1",
    )
    chassis.visual(
        Box((front_t, bay_width, case_height - bay_max_z)),
        origin=Origin(xyz=(front_center_x, 0.0, (case_height + bay_max_z) / 2.0)),
        material="aged_beige",
        name="front_upper_panel",
    )
    chassis.visual(
        Box((front_t, bay_width, bay_min_z)),
        origin=Origin(xyz=(front_center_x, 0.0, bay_min_z / 2.0)),
        material="aged_beige",
        name="front_lower_panel",
    )

    drive_trim = BezelGeometry(
        opening_size=(bay_height, bay_width),
        outer_size=(0.074, 0.178),
        depth=0.010,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.003,
        outer_corner_radius=0.007,
        face=BezelFace(style="radiused_step", front_lip=0.002, fillet=0.0015),
    )
    chassis.visual(
        mesh_from_geometry(drive_trim, "drive_bay_trim"),
        origin=Origin(xyz=(front_x + 0.005, 0.0, bay_center_z), rpy=(0.0, pi / 2.0, 0.0)),
        material="slightly_darker_beige",
        name="drive_bay_trim",
    )

    # Dark internal guide rails make the bay read as a usable slot for the tray.
    chassis.visual(
        Box((0.166, 0.006, 0.012)),
        origin=Origin(xyz=(0.125, -0.074, bay_min_z + 0.011)),
        material="warm_shadow",
        name="bay_side_rail_0",
    )
    chassis.visual(
        Box((0.166, 0.006, 0.012)),
        origin=Origin(xyz=(0.125, 0.074, bay_min_z + 0.011)),
        material="warm_shadow",
        name="bay_side_rail_1",
    )
    chassis.visual(
        Box((0.155, 0.144, 0.004)),
        origin=Origin(xyz=(0.128, 0.0, bay_max_z - 0.002)),
        material="warm_shadow",
        name="bay_upper_shadow",
    )

    # Retro front details below the optical bay: a darker 3.5-inch blank, LEDs,
    # rubber feet, and a small model badge.  They are flush-mounted onto the
    # connected front panel so they do not read as floating decorations.
    chassis.visual(
        Box((0.006, 0.104, 0.028)),
        origin=Origin(xyz=(front_x + 0.003, 0.0, 0.286)),
        material="slightly_darker_beige",
        name="floppy_blank",
    )
    chassis.visual(
        Box((0.005, 0.078, 0.017)),
        origin=Origin(xyz=(front_x + 0.0025, 0.0, 0.246)),
        material="dark_drive_plastic",
        name="dark_nameplate",
    )
    chassis.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(front_x + 0.002, -0.050, 0.190), rpy=(0.0, pi / 2.0, 0.0)),
        material="green_led",
        name="power_led",
    )
    chassis.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(front_x + 0.002, -0.030, 0.190), rpy=(0.0, pi / 2.0, 0.0)),
        material="amber_led",
        name="disk_led",
    )
    for y_pos in (-0.066, 0.066):
        chassis.visual(
            Box((0.028, 0.032, 0.010)),
            origin=Origin(xyz=(0.135, y_pos, 0.005)),
            material="rubber_black",
            name=f"front_foot_{0 if y_pos < 0 else 1}",
        )

    # Exposed hinge support attached to the rear corner of the chassis.
    hinge_x = rear_x - 0.002
    hinge_y = side_y + 0.019
    hinge_z = 0.230
    chassis.visual(
        Box((0.008, 0.024, 0.420)),
        origin=Origin(xyz=(rear_x - 0.002, side_y + 0.012, hinge_z)),
        material="brushed_metal",
        name="side_hinge_backer",
    )
    for idx, z_pos in enumerate((0.095, 0.230, 0.365)):
        chassis.visual(
            Cylinder(radius=0.004, length=0.075),
            origin=Origin(xyz=(hinge_x, hinge_y, z_pos)),
            material="brushed_metal",
            name=f"side_hinge_barrel_{idx}",
        )

    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((0.402, 0.010, 0.420)),
        origin=Origin(xyz=(0.209, -0.013, 0.0)),
        material="aged_beige",
        name="panel_skin",
    )
    side_panel.visual(
        Box((0.014, 0.004, 0.400)),
        origin=Origin(xyz=(0.007, -0.006, 0.0)),
        material="brushed_metal",
        name="panel_hinge_leaf",
    )
    side_vent = VentGrilleGeometry(
        (0.150, 0.086),
        frame=0.010,
        face_thickness=0.004,
        duct_depth=0.006,
        slat_pitch=0.014,
        slat_width=0.006,
        slat_angle_deg=25.0,
        corner_radius=0.004,
        slats=VentGrilleSlats(profile="flat", direction="down", divider_count=1, divider_width=0.003),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.001),
        sleeve=VentGrilleSleeve(style="none"),
    )
    side_panel.visual(
        mesh_from_geometry(side_vent, "side_vent_grille"),
        origin=Origin(xyz=(0.275, -0.007, 0.055), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="slightly_darker_beige",
        name="side_vent_grille",
    )
    for idx, (x_pos, z_pos) in enumerate(((0.060, 0.180), (0.360, 0.180), (0.060, -0.180), (0.360, -0.180))):
        side_panel.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(x_pos, -0.007, z_pos), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="brushed_metal",
            name=f"panel_screw_{idx}",
        )

    drive_tray = model.part("drive_tray")
    drive_tray.visual(
        Box((0.170, 0.130, 0.008)),
        origin=Origin(xyz=(-0.085, 0.0, 0.0)),
        material="dark_drive_plastic",
        name="tray_slab",
    )
    for idx, y_pos in enumerate((-0.066, 0.066)):
        drive_tray.visual(
            Box((0.160, 0.006, 0.016)),
            origin=Origin(xyz=(-0.085, -0.068 if y_pos < 0 else 0.068, 0.006)),
            material="dark_drive_plastic",
            name=f"tray_raised_rail_{idx}",
        )
    drive_tray.visual(
        Box((0.050, 0.080, 0.002)),
        origin=Origin(xyz=(-0.060, 0.0, 0.005)),
        material="brushed_metal",
        name="tray_disc_recess",
    )

    drive_bezel = model.part("drive_bezel")
    drive_bezel.visual(
        Box((0.006, 0.146, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material="aged_beige",
        name="bezel_plate",
    )
    drive_bezel.visual(
        Cylinder(radius=0.004, length=0.144),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="bezel_hinge_pin",
    )
    drive_bezel.visual(
        Box((0.004, 0.050, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.033)),
        material="slightly_darker_beige",
        name="bezel_pull_lip",
    )

    tray_origin_x = front_x - 0.002
    tray_origin_z = bay_min_z + 0.013
    model.articulation(
        "side_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=drive_tray,
        origin=Origin(xyz=(tray_origin_x, 0.0, tray_origin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.22, lower=0.0, upper=0.130),
    )
    model.articulation(
        "tray_bezel_hinge",
        ArticulationType.REVOLUTE,
        parent=drive_tray,
        child=drive_bezel,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    drive_tray = object_model.get_part("drive_tray")
    drive_bezel = object_model.get_part("drive_bezel")
    side_hinge = object_model.get_articulation("side_hinge")
    tray_slide = object_model.get_articulation("tray_slide")
    bezel_hinge = object_model.get_articulation("tray_bezel_hinge")

    ctx.expect_gap(
        side_panel,
        chassis,
        axis="y",
        positive_elem="panel_skin",
        negative_elem="top_panel",
        min_gap=0.0,
        max_gap=0.004,
        name="closed side panel sits just outside chassis shell",
    )
    ctx.expect_overlap(
        drive_tray,
        chassis,
        axes="x",
        elem_a="tray_slab",
        elem_b="bay_side_rail_0",
        min_overlap=0.140,
        name="retracted tray is captured in the 5.25 bay rails",
    )
    ctx.expect_within(
        drive_bezel,
        chassis,
        axes="yz",
        inner_elem="bezel_plate",
        outer_elem="drive_bay_trim",
        margin=0.004,
        name="closed flip bezel fits inside the 5.25 bay trim",
    )

    closed_side_aabb = ctx.part_element_world_aabb(side_panel, elem="panel_skin")
    with ctx.pose({side_hinge: 1.20}):
        open_side_aabb = ctx.part_element_world_aabb(side_panel, elem="panel_skin")
    ctx.check(
        "side panel swings outward on rear vertical hinge",
        closed_side_aabb is not None
        and open_side_aabb is not None
        and open_side_aabb[1][1] > closed_side_aabb[1][1] + 0.20,
        details=f"closed={closed_side_aabb}, open={open_side_aabb}",
    )

    retracted_tray_aabb = ctx.part_element_world_aabb(drive_tray, elem="tray_slab")
    with ctx.pose({tray_slide: 0.130}):
        extended_tray_aabb = ctx.part_element_world_aabb(drive_tray, elem="tray_slab")
        ctx.expect_overlap(
            drive_tray,
            chassis,
            axes="x",
            elem_a="tray_slab",
            elem_b="bay_side_rail_0",
            min_overlap=0.025,
            name="extended tray retains insertion in bay rails",
        )
    ctx.check(
        "tray slide moves outward from the front",
        retracted_tray_aabb is not None
        and extended_tray_aabb is not None
        and extended_tray_aabb[1][0] > retracted_tray_aabb[1][0] + 0.12,
        details=f"retracted={retracted_tray_aabb}, extended={extended_tray_aabb}",
    )

    closed_bezel_aabb = ctx.part_element_world_aabb(drive_bezel, elem="bezel_plate")
    with ctx.pose({bezel_hinge: 1.35}):
        open_bezel_aabb = ctx.part_element_world_aabb(drive_bezel, elem="bezel_plate")
    ctx.check(
        "drive bezel flips down and forward",
        closed_bezel_aabb is not None
        and open_bezel_aabb is not None
        and open_bezel_aabb[1][0] > closed_bezel_aabb[1][0] + 0.030
        and open_bezel_aabb[1][2] < closed_bezel_aabb[1][2] - 0.020,
        details=f"closed={closed_bezel_aabb}, open={open_bezel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
