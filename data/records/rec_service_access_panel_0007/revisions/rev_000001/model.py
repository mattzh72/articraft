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
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

OUTER_W = 0.74
OUTER_H = 0.86
OPEN_W = 0.44
OPEN_H = 0.552
DOOR_W = 0.42
DOOR_H = 0.544
FACE_T = 0.006
BEZEL_T = 0.002
BODY_D = 0.058
BACK_T = 0.006
HINGE_R = 0.008
HINGE_X = -0.208
HINGE_Y = -0.001
DOOR_CENTER_X = 0.006
DOOR_LOCAL_X = DOOR_CENTER_X - HINGE_X


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_rect_solid_mesh(
    name: str,
    *,
    width: float,
    height: float,
    depth: float,
    radius: float,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        height=depth,
        center=True,
        cap=True,
        closed=True,
    ).rotate_x(-math.pi / 2.0)
    return _save_mesh(name, geom)


def _rounded_rect_ring_mesh(
    name: str,
    *,
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    depth: float,
    outer_radius: float,
    inner_radius: float,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_height, outer_radius, corner_segments=8),
        [rounded_rect_profile(inner_width, inner_height, inner_radius, corner_segments=8)],
        height=depth,
        center=True,
        cap=True,
        closed=True,
    ).rotate_x(-math.pi / 2.0)
    return _save_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_service_access_panel", assets=ASSETS)

    enclosure_matte = model.material("enclosure_matte", rgba=(0.17, 0.18, 0.19, 1.0))
    panel_matte = model.material("panel_matte", rgba=(0.24, 0.25, 0.27, 1.0))
    satin_trim = model.material("satin_trim", rgba=(0.60, 0.62, 0.65, 1.0))
    satin_hardware = model.material("satin_hardware", rgba=(0.40, 0.42, 0.45, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.07, 0.08, 0.09, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.09, 0.10, 0.11, 1.0))

    fascia_mesh = _rounded_rect_ring_mesh(
        "service_panel_fascia.obj",
        outer_width=OUTER_W,
        outer_height=OUTER_H,
        inner_width=OPEN_W,
        inner_height=OPEN_H,
        depth=FACE_T,
        outer_radius=0.038,
        inner_radius=0.024,
    )
    bezel_mesh = _rounded_rect_ring_mesh(
        "service_panel_bezel.obj",
        outer_width=OPEN_W + 0.040,
        outer_height=OPEN_H + 0.040,
        inner_width=OPEN_W,
        inner_height=OPEN_H,
        depth=BEZEL_T,
        outer_radius=0.026,
        inner_radius=0.024,
    )
    gasket_mesh = _rounded_rect_ring_mesh(
        "service_panel_gasket_land.obj",
        outer_width=OPEN_W,
        outer_height=OPEN_H,
        inner_width=DOOR_W + 0.006,
        inner_height=DOOR_H + 0.008,
        depth=BEZEL_T,
        outer_radius=0.024,
        inner_radius=0.018,
    )
    door_skin_mesh = _rounded_rect_solid_mesh(
        "service_panel_door_skin.obj",
        width=DOOR_W,
        height=DOOR_H,
        depth=0.006,
        radius=0.018,
    )
    door_tray_mesh = _rounded_rect_solid_mesh(
        "service_panel_door_tray.obj",
        width=0.402,
        height=0.526,
        depth=0.014,
        radius=0.014,
    )

    enclosure = model.part("enclosure_face")
    enclosure.visual(
        fascia_mesh,
        origin=Origin(xyz=(0.0, -FACE_T * 0.5, 0.0)),
        material=enclosure_matte,
        name="fascia_face",
    )
    enclosure.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, -0.001, 0.0)),
        material=satin_trim,
        name="opening_bezel",
    )
    enclosure.visual(
        Box(((OUTER_W - OPEN_W) * 0.5, BODY_D, OUTER_H)),
        origin=Origin(xyz=(-(OUTER_W + OPEN_W) * 0.25, -(FACE_T + BODY_D) * 0.5, 0.0)),
        material=enclosure_matte,
        name="left_stile",
    )
    enclosure.visual(
        Box(((OUTER_W - OPEN_W) * 0.5, BODY_D, OUTER_H)),
        origin=Origin(xyz=((OUTER_W + OPEN_W) * 0.25, -(FACE_T + BODY_D) * 0.5, 0.0)),
        material=enclosure_matte,
        name="right_stile",
    )
    enclosure.visual(
        Box((OPEN_W, BODY_D, (OUTER_H - OPEN_H) * 0.5)),
        origin=Origin(xyz=(0.0, -(FACE_T + BODY_D) * 0.5, (OUTER_H + OPEN_H) * 0.25)),
        material=enclosure_matte,
        name="top_rail",
    )
    enclosure.visual(
        Box((OPEN_W, BODY_D, (OUTER_H - OPEN_H) * 0.5)),
        origin=Origin(xyz=(0.0, -(FACE_T + BODY_D) * 0.5, -(OUTER_H + OPEN_H) * 0.25)),
        material=enclosure_matte,
        name="bottom_rail",
    )
    enclosure.visual(
        gasket_mesh,
        origin=Origin(xyz=(0.0, -(FACE_T + BEZEL_T * 0.5), 0.0)),
        material=gasket_black,
        name="gasket_land",
    )
    enclosure.visual(
        Box((0.520, BACK_T, 0.660)),
        origin=Origin(xyz=(0.0, -(FACE_T + BODY_D - BACK_T * 0.5), 0.0)),
        material=shadow_black,
        name="cavity_back",
    )
    enclosure.visual(
        Box((0.144, 0.003, 0.066)),
        origin=Origin(xyz=(0.046, -(FACE_T + BODY_D - 0.0015), -0.160)),
        material=satin_trim,
        name="service_id_plate",
    )
    enclosure.visual(
        Box((0.016, 0.014, 0.112)),
        origin=Origin(xyz=(HINGE_X - 0.014, -0.008, 0.200)),
        material=enclosure_matte,
        name="upper_hinge_lug",
    )
    enclosure.visual(
        Box((0.016, 0.014, 0.112)),
        origin=Origin(xyz=(HINGE_X - 0.014, -0.008, -0.200)),
        material=enclosure_matte,
        name="lower_hinge_lug",
    )
    enclosure.visual(
        Cylinder(radius=HINGE_R, length=0.096),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.200)),
        material=satin_trim,
        name="frame_barrel_upper",
    )
    enclosure.visual(
        Cylinder(radius=HINGE_R, length=0.096),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, -0.200)),
        material=satin_trim,
        name="frame_barrel_lower",
    )
    enclosure.visual(
        Box((0.014, 0.018, 0.052)),
        origin=Origin(xyz=(0.227, -0.016, 0.176)),
        material=satin_hardware,
        name="strike_pad_upper",
    )
    enclosure.visual(
        Box((0.014, 0.018, 0.052)),
        origin=Origin(xyz=(0.227, -0.016, -0.176)),
        material=satin_hardware,
        name="strike_pad_lower",
    )

    for index, (x_pos, z_pos) in enumerate(
        (
            (-0.286, 0.314),
            (0.286, 0.314),
            (-0.286, -0.314),
            (0.286, -0.314),
        )
    ):
        enclosure.visual(
            Cylinder(radius=0.009, length=0.003),
            origin=Origin(xyz=(x_pos, -0.0015, z_pos), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_hardware,
            name=f"frame_fastener_{index}",
        )
        enclosure.visual(
            Box((0.014, 0.0014, 0.0022)),
            origin=Origin(xyz=(x_pos, -0.0008, z_pos)),
            material=shadow_black,
            name=f"frame_fastener_slot_{index}",
        )

    enclosure.inertial = Inertial.from_geometry(
        Box((OUTER_W, FACE_T + BODY_D + BACK_T, OUTER_H)),
        mass=12.5,
        origin=Origin(xyz=(0.0, -(FACE_T + BODY_D + BACK_T) * 0.5, 0.0)),
    )

    panel = model.part("access_panel")
    panel.visual(
        door_skin_mesh,
        origin=Origin(xyz=(DOOR_LOCAL_X, -0.0035, 0.0)),
        material=panel_matte,
        name="door_skin",
    )
    panel.visual(
        door_tray_mesh,
        origin=Origin(xyz=(DOOR_LOCAL_X, -0.012, 0.0)),
        material=panel_matte,
        name="door_tray",
    )
    panel.visual(
        Box((0.306, 0.010, 0.392)),
        origin=Origin(xyz=(DOOR_LOCAL_X + 0.012, -0.018, 0.0)),
        material=shadow_black,
        name="inner_service_plate",
    )
    panel.visual(
        Box((0.020, 0.010, 0.460)),
        origin=Origin(xyz=(0.010, -0.010, 0.0)),
        material=satin_trim,
        name="hinge_leaf",
    )
    panel.visual(
        Cylinder(radius=HINGE_R, length=0.304),
        origin=Origin(xyz=(0.0, HINGE_Y, 0.0)),
        material=satin_trim,
        name="panel_barrel",
    )
    panel.visual(
        Box((0.010, 0.012, 0.420)),
        origin=Origin(xyz=(DOOR_LOCAL_X + 0.196, -0.012, 0.0)),
        material=satin_trim,
        name="latch_rail",
    )
    panel.visual(
        Box((0.100, 0.008, 0.034)),
        origin=Origin(xyz=(DOOR_LOCAL_X + 0.020, -0.020, 0.172)),
        material=shadow_black,
        name="upper_reinforcement_rib",
    )
    panel.visual(
        Box((0.100, 0.008, 0.034)),
        origin=Origin(xyz=(DOOR_LOCAL_X + 0.020, -0.020, -0.172)),
        material=shadow_black,
        name="lower_reinforcement_rib",
    )
    for name, z_pos in (("latch_head_upper", 0.176), ("latch_head_lower", -0.176)):
        panel.visual(
            Cylinder(radius=0.011, length=0.003),
            origin=Origin(
                xyz=(DOOR_LOCAL_X + 0.182, -0.0015, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=satin_hardware,
            name=name,
        )
        panel.visual(
            Box((0.015, 0.0015, 0.0024)),
            origin=Origin(xyz=(DOOR_LOCAL_X + 0.182, -0.0008, z_pos)),
            material=shadow_black,
            name=f"{name}_slot",
        )

    panel.inertial = Inertial.from_geometry(
        Box((DOOR_W, 0.026, DOOR_H)),
        mass=4.0,
        origin=Origin(xyz=(DOOR_LOCAL_X, -0.013, 0.0)),
    )

    model.articulation(
        "service_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=panel,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    enclosure = object_model.get_part("enclosure_face")
    panel = object_model.get_part("access_panel")
    hinge = object_model.get_articulation("service_panel_hinge")

    fascia_face = enclosure.get_visual("fascia_face")
    opening_bezel = enclosure.get_visual("opening_bezel")
    frame_barrel_upper = enclosure.get_visual("frame_barrel_upper")
    frame_barrel_lower = enclosure.get_visual("frame_barrel_lower")
    strike_pad_upper = enclosure.get_visual("strike_pad_upper")
    strike_pad_lower = enclosure.get_visual("strike_pad_lower")

    door_skin = panel.get_visual("door_skin")
    panel_barrel = panel.get_visual("panel_barrel")
    latch_head_upper = panel.get_visual("latch_head_upper")
    latch_head_lower = panel.get_visual("latch_head_lower")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=18,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            panel,
            enclosure,
            elem_a=panel_barrel,
            elem_b=frame_barrel_upper,
            name="upper_hinge_knuckle_contact_closed",
        )
        ctx.expect_contact(
            panel,
            enclosure,
            elem_a=panel_barrel,
            elem_b=frame_barrel_lower,
            name="lower_hinge_knuckle_contact_closed",
        )
        ctx.expect_overlap(
            panel,
            enclosure,
            axes="x",
            min_overlap=0.41,
            elem_a=door_skin,
            elem_b=opening_bezel,
            name="panel_width_reads_framed_in_closed_pose",
        )
        ctx.expect_overlap(
            panel,
            enclosure,
            axes="z",
            min_overlap=0.53,
            elem_a=door_skin,
            elem_b=opening_bezel,
            name="panel_height_reads_framed_in_closed_pose",
        )
        ctx.expect_within(
            panel,
            enclosure,
            axes="xz",
            margin=0.0,
            inner_elem=door_skin,
            outer_elem=opening_bezel,
            name="panel_stays_inside_bezel_outer_envelope",
        )
        ctx.expect_origin_distance(
            panel,
            enclosure,
            axes="y",
            max_dist=0.01,
            name="closed_panel_keeps_tight_face_plane_alignment",
        )

    with ctx.pose({hinge: 1.25}):
        ctx.expect_contact(
            panel,
            enclosure,
            elem_a=panel_barrel,
            elem_b=frame_barrel_upper,
            name="upper_hinge_knuckle_contact_open",
        )
        ctx.expect_contact(
            panel,
            enclosure,
            elem_a=panel_barrel,
            elem_b=frame_barrel_lower,
            name="lower_hinge_knuckle_contact_open",
        )
        ctx.expect_gap(
            panel,
            enclosure,
            axis="y",
            min_gap=0.05,
            positive_elem=latch_head_upper,
            negative_elem=fascia_face,
            name="upper_latch_swings_clear_of_enclosure",
        )
        ctx.expect_gap(
            panel,
            enclosure,
            axis="y",
            min_gap=0.05,
            positive_elem=latch_head_lower,
            negative_elem=fascia_face,
            name="lower_latch_swings_clear_of_enclosure",
        )
        ctx.expect_overlap(
            panel,
            enclosure,
            axes="z",
            min_overlap=0.020,
            elem_a=latch_head_upper,
            elem_b=strike_pad_upper,
            name="upper_latch_stays_at_strike_height_when_open",
        )
        ctx.expect_overlap(
            panel,
            enclosure,
            axes="z",
            min_overlap=0.020,
            elem_a=latch_head_lower,
            elem_b=strike_pad_lower,
            name="lower_latch_stays_at_strike_height_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
