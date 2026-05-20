from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    VentGrilleGeometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def grille_panel_geometry(
        panel_size,
        thickness,
        *,
        frame,
        slat_pitch,
        slat_width,
        slat_angle_deg,
        corner_radius,
        center=True,
    ):
        geom = VentGrilleGeometry(
            panel_size,
            frame=frame,
            face_thickness=thickness,
            duct_depth=max(0.0015, thickness * 0.75),
            duct_wall=max(0.001, min(frame * 0.45, thickness * 0.75)),
            slat_pitch=slat_pitch,
            slat_width=slat_width,
            slat_angle_deg=slat_angle_deg,
            corner_radius=corner_radius,
        )
        if not center:
            geom.translate(0.0, 0.0, thickness * 0.5)
        return geom

    def rect_loop(width: float, depth: float, z: float, radius: float):
        profile = rounded_rect_profile(width, depth, radius, corner_segments=8)
        return [(x, y, z) for x, y in profile]

    def rounded_vertical_panel(width: float, height: float, thickness: float, radius: float):
        return ExtrudeGeometry.centered(
            rounded_rect_profile(width, height, radius, corner_segments=8),
            thickness,
        ).rotate_x(-math.pi / 2.0)

    def rounded_horizontal_plate(width: float, depth: float, thickness: float, radius: float):
        return ExtrudeGeometry.centered(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            thickness,
        )

    model = ArticulatedObject(name="outdoor_tower_air_purifier", assets=ASSETS)

    shell_powder = model.material("shell_powder", rgba=(0.78, 0.80, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    deep_black = model.material("deep_black", rgba=(0.09, 0.10, 0.10, 1.0))
    hood_black = model.material("hood_black", rgba=(0.13, 0.14, 0.15, 1.0))
    filter_olive = model.material("filter_olive", rgba=(0.41, 0.46, 0.34, 1.0))
    lens_tint = model.material("lens_tint", rgba=(0.38, 0.62, 0.51, 0.80))

    tower = model.part("tower_body")
    tower.inertial = Inertial.from_geometry(
        Box((0.34, 0.28, 1.20)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )

    body_shell = repair_loft(
        section_loft(
            [
                rect_loop(0.30, 0.24, 0.06, 0.038),
                rect_loop(0.30, 0.24, 0.20, 0.038),
                rect_loop(0.29, 0.23, 0.62, 0.036),
                rect_loop(0.27, 0.21, 0.98, 0.032),
                rect_loop(0.25, 0.19, 1.08, 0.028),
            ]
        )
    )
    tower.visual(
        save_mesh("tower_body_shell.obj", body_shell),
        material=shell_powder,
        name="body_shell",
    )
    tower.visual(
        save_mesh("tower_base_plinth.obj", rounded_horizontal_plate(0.34, 0.28, 0.04, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=charcoal,
        name="base_plinth",
    )
    for name, xyz in {
        "foot_front_left": (-0.115, 0.090, 0.01),
        "foot_front_right": (0.115, 0.090, 0.01),
        "foot_rear_left": (-0.115, -0.090, 0.01),
        "foot_rear_right": (0.115, -0.090, 0.01),
    }.items():
        tower.visual(Box((0.045, 0.050, 0.020)), origin=Origin(xyz=xyz), material=deep_black, name=name)

    tower.visual(
        save_mesh("tower_filter_cartridge.obj", rounded_vertical_panel(0.160, 0.650, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.100, 0.54)),
        material=filter_olive,
        name="filter_cartridge",
    )
    tower.visual(
        save_mesh("tower_intake_cavity.obj", rounded_vertical_panel(0.172, 0.694, 0.060, 0.014)),
        origin=Origin(xyz=(0.0, 0.078, 0.54)),
        material=deep_black,
        name="intake_cavity",
    )
    tower.visual(
        save_mesh("tower_status_window.obj", rounded_vertical_panel(0.056, 0.138, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.116, 0.965)),
        material=lens_tint,
        name="status_window",
    )
    tower.visual(
        Box((0.050, 0.012, 0.132)),
        origin=Origin(xyz=(0.0, 0.108, 0.965)),
        material=deep_black,
        name="status_window_bezel",
    )

    top_deck = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.216, 0.156, 0.024, corner_segments=8),
        [rounded_rect_profile(0.150, 0.090, 0.012, corner_segments=8)],
        height=0.020,
        center=True,
    )
    tower.visual(
        save_mesh("tower_top_deck.obj", top_deck),
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
        material=shell_powder,
        name="top_deck",
    )
    tower.visual(
        Box((0.146, 0.086, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, 1.046)),
        material=deep_black,
        name="exhaust_cavity",
    )
    tower.visual(
        save_mesh(
            "tower_exhaust_grille.obj",
            grille_panel_geometry(
                panel_size=(0.146, 0.086),
                thickness=0.010,
                frame=0.010,
                slat_pitch=0.020,
                slat_width=0.008,
                slat_angle_deg=32.0,
                corner_radius=0.008,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.098)),
        material=charcoal,
        name="exhaust_grille",
    )
    tower.visual(
        save_mesh(
            "tower_exhaust_grille_seat.obj",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.156, 0.096, 0.010, corner_segments=8),
                [rounded_rect_profile(0.132, 0.072, 0.008, corner_segments=8)],
                height=0.010,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.094)),
        material=deep_black,
        name="exhaust_grille_seat",
    )
    tower.visual(
        Box((0.014, 0.024, 0.074)),
        origin=Origin(xyz=(-0.108, 0.121, 0.40)),
        material=charcoal,
        name="body_hinge_leaf_lower",
    )
    tower.visual(
        Box((0.014, 0.024, 0.074)),
        origin=Origin(xyz=(-0.121, 0.121, 0.54)),
        material=charcoal,
        name="body_hinge_mount_mid",
    )
    tower.visual(
        Box((0.014, 0.024, 0.074)),
        origin=Origin(xyz=(-0.108, 0.121, 0.68)),
        material=charcoal,
        name="body_hinge_leaf_upper",
    )
    for name, xyz in {
        "hood_post_front_left": (-0.082, 0.056, 1.12),
        "hood_post_front_right": (0.082, 0.056, 1.12),
        "hood_post_rear_left": (-0.082, -0.056, 1.12),
        "hood_post_rear_right": (0.082, -0.056, 1.12),
    }.items():
        tower.visual(Box((0.020, 0.020, 0.040)), origin=Origin(xyz=xyz), material=hood_black, name=name)
    tower.visual(
        save_mesh("tower_top_hood.obj", rounded_horizontal_plate(0.280, 0.220, 0.018, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 1.149)),
        material=hood_black,
        name="top_hood",
    )
    tower.visual(
        Cylinder(radius=0.006, length=0.10),
        origin=Origin(xyz=(-0.102, 0.133, 0.40)),
        material=charcoal,
        name="body_hinge_lower",
    )
    tower.visual(
        Cylinder(radius=0.006, length=0.10),
        origin=Origin(xyz=(-0.102, 0.133, 0.68)),
        material=charcoal,
        name="body_hinge_upper",
    )

    door = model.part("intake_door")
    door.inertial = Inertial.from_geometry(
        Box((0.204, 0.018, 0.760)),
        mass=2.5,
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
    )
    door_frame = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.194, 0.760, 0.020, corner_segments=8),
        [rounded_rect_profile(0.166, 0.684, 0.010, corner_segments=8)],
        height=0.012,
        center=True,
    ).rotate_x(-math.pi / 2.0)
    door.visual(
        save_mesh("tower_intake_door_frame.obj", door_frame),
        origin=Origin(xyz=(0.106, 0.0, 0.0)),
        material=shell_powder,
        name="door_frame",
    )
    door.visual(
        save_mesh(
            "tower_intake_door_grille.obj",
            grille_panel_geometry(
                panel_size=(0.162, 0.680),
                thickness=0.009,
                frame=0.008,
                slat_pitch=0.024,
                slat_width=0.010,
                slat_angle_deg=28.0,
                corner_radius=0.010,
                center=True,
            ).rotate_x(-math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.102, -0.001, 0.0)),
        material=charcoal,
        name="door_grille",
    )
    door.visual(
        Box((0.016, 0.020, 0.074)),
        origin=Origin(xyz=(0.008, 0.0, -0.28)),
        material=charcoal,
        name="door_hinge_leaf_lower",
    )
    door.visual(
        Box((0.016, 0.020, 0.074)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=charcoal,
        name="door_hinge_leaf_mid",
    )
    door.visual(
        Box((0.016, 0.020, 0.074)),
        origin=Origin(xyz=(0.008, 0.0, 0.28)),
        material=charcoal,
        name="door_hinge_leaf_upper",
    )
    door.visual(
        save_mesh("tower_door_handle.obj", rounded_vertical_panel(0.018, 0.120, 0.018, 0.004)),
        origin=Origin(xyz=(0.184, 0.006, 0.0)),
        material=deep_black,
        name="door_handle",
    )
    for name, z_center in {
        "door_knuckle_lower": -0.28,
        "door_knuckle_mid": 0.0,
        "door_knuckle_upper": 0.28,
    }.items():
        door.visual(
            Cylinder(radius=0.006, length=0.18),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=charcoal,
            name=name,
        )

    model.articulation(
        "intake_door_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=door,
        origin=Origin(xyz=(-0.102, 0.133, 0.54)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower_body")
    door = object_model.get_part("intake_door")
    hinge = object_model.get_articulation("intake_door_hinge")
    body_shell = tower.get_visual("body_shell")
    base_plinth = tower.get_visual("base_plinth")
    filter_cartridge = tower.get_visual("filter_cartridge")
    top_hood = tower.get_visual("top_hood")
    exhaust_grille = tower.get_visual("exhaust_grille")
    body_hinge_lower = tower.get_visual("body_hinge_lower")
    door_frame = door.get_visual("door_frame")
    door_grille = door.get_visual("door_grille")
    door_knuckle_lower = door.get_visual("door_knuckle_lower")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(door, tower, axes="xz", min_overlap=0.10, elem_a=door_frame, elem_b=body_shell)
    ctx.expect_gap(
        door,
        tower,
        axis="y",
        max_gap=0.020,
        max_penetration=0.0,
        positive_elem=door_grille,
        negative_elem=filter_cartridge,
    )
    ctx.expect_within(
        tower,
        door,
        axes="xz",
        inner_elem=filter_cartridge,
        outer_elem=door_grille,
    )
    ctx.expect_contact(door, tower, elem_a=door_knuckle_lower, elem_b=body_hinge_lower)
    ctx.expect_overlap(
        tower,
        tower,
        axes="xy",
        elem_a=top_hood,
        elem_b=body_shell,
        min_overlap=0.18,
    )
    ctx.expect_within(
        tower,
        tower,
        axes="xy",
        inner_elem=exhaust_grille,
        outer_elem=top_hood,
    )
    ctx.expect_gap(
        tower,
        tower,
        axis="z",
        max_gap=0.050,
        max_penetration=0.0,
        positive_elem=top_hood,
        negative_elem=exhaust_grille,
    )
    ctx.expect_gap(
        tower,
        tower,
        axis="z",
        min_gap=1.00,
        positive_elem=top_hood,
        negative_elem=base_plinth,
    )
    with ctx.pose({hinge: 1.45}):
        ctx.expect_gap(
            door,
            tower,
            axis="y",
            min_gap=0.010,
            positive_elem=door_frame,
            negative_elem=filter_cartridge,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
