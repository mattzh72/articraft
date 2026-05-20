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

ENCLOSURE_W = 0.78
ENCLOSURE_H = 1.00
FACE_T = 0.018
OPENING_W = 0.44
OPENING_H = 0.62
OPENING_R = 0.020
OPENING_X = 0.040
OPENING_Z = 0.000
DOOR_W = 0.432
DOOR_H = 0.612
DOOR_R = 0.016
DOOR_T = 0.012
OPENING_LEFT = OPENING_X - OPENING_W / 2.0
OPENING_RIGHT = OPENING_X + OPENING_W / 2.0
HINGE_AXIS_X = OPENING_LEFT - 0.005
HINGE_AXIS_Y = 0.015
DOOR_LEFT_FROM_AXIS = (OPENING_LEFT + 0.004) - HINGE_AXIS_X
DOOR_CENTER_X = DOOR_LEFT_FROM_AXIS + DOOR_W / 2.0
LINER_DEPTH = 0.064
PANEL_BODY_Y = -0.012
PANEL_TRIM_Y = -0.00725
PANEL_INSERT_Y = -0.0105


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _panel_mesh(
    *,
    filename: str,
    outer: list[tuple[float, float]],
    thickness: float,
    holes: list[list[tuple[float, float]]] | None = None,
):
    if holes:
        geom = ExtrudeWithHolesGeometry(
            outer_profile=outer,
            hole_profiles=holes,
            height=thickness,
            center=True,
            cap=True,
            closed=True,
        )
    else:
        geom = ExtrudeGeometry(
            outer,
            height=thickness,
            center=True,
            cap=True,
            closed=True,
        )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_dir / filename)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_service_access_panel", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.26, 0.28, 0.31, 1.0))
    satin_titanium = model.material("satin_titanium", rgba=(0.53, 0.55, 0.58, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.07, 0.08, 0.09, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))

    enclosure_outer = rounded_rect_profile(ENCLOSURE_W, ENCLOSURE_H, radius=0.034, corner_segments=8)
    opening_profile = _offset_profile(
        rounded_rect_profile(OPENING_W, OPENING_H, radius=OPENING_R, corner_segments=8),
        OPENING_X,
        OPENING_Z,
    )
    outer_bezel_inner = rounded_rect_profile(
        ENCLOSURE_W - 0.070,
        ENCLOSURE_H - 0.070,
        radius=0.026,
        corner_segments=8,
    )
    service_frame_outer = _offset_profile(
        rounded_rect_profile(
            OPENING_W + 0.090,
            OPENING_H + 0.096,
            radius=OPENING_R + 0.014,
            corner_segments=8,
        ),
        OPENING_X,
        OPENING_Z,
    )

    door_outer = rounded_rect_profile(DOOR_W, DOOR_H, radius=DOOR_R, corner_segments=8)
    door_trim_inner = rounded_rect_profile(
        DOOR_W - 0.100,
        DOOR_H - 0.114,
        radius=max(0.004, DOOR_R - 0.008),
        corner_segments=8,
    )
    door_insert = rounded_rect_profile(
        DOOR_W - 0.124,
        DOOR_H - 0.144,
        radius=max(0.004, DOOR_R - 0.010),
        corner_segments=8,
    )

    face_plate_mesh = _panel_mesh(
        filename="enclosure_face_plate.obj",
        outer=enclosure_outer,
        holes=[opening_profile],
        thickness=FACE_T,
    )
    outer_bezel_mesh = _panel_mesh(
        filename="enclosure_outer_bezel.obj",
        outer=enclosure_outer,
        holes=[outer_bezel_inner],
        thickness=0.0028,
    )
    service_frame_mesh = _panel_mesh(
        filename="service_frame_ring.obj",
        outer=service_frame_outer,
        holes=[opening_profile],
        thickness=0.0030,
    )
    door_body_mesh = _panel_mesh(
        filename="access_panel_body.obj",
        outer=door_outer,
        thickness=DOOR_T,
    )
    door_trim_mesh = _panel_mesh(
        filename="access_panel_trim.obj",
        outer=rounded_rect_profile(
            DOOR_W - 0.018,
            DOOR_H - 0.020,
            radius=max(0.004, DOOR_R - 0.002),
            corner_segments=8,
        ),
        holes=[door_trim_inner],
        thickness=0.0025,
    )
    door_insert_mesh = _panel_mesh(
        filename="access_panel_insert.obj",
        outer=door_insert,
        thickness=0.0040,
    )

    enclosure = model.part("enclosure")
    enclosure.visual(
        face_plate_mesh,
        material=matte_graphite,
        name="face_plate",
    )
    enclosure.visual(
        outer_bezel_mesh,
        origin=Origin(xyz=(0.0, FACE_T / 2.0 + 0.0014, 0.0)),
        material=satin_graphite,
        name="outer_bezel",
    )
    enclosure.visual(
        service_frame_mesh,
        origin=Origin(xyz=(0.0, FACE_T / 2.0 + 0.0015, 0.0)),
        material=satin_titanium,
        name="service_frame",
    )

    wall_y = -(FACE_T / 2.0 + LINER_DEPTH / 2.0)
    side_wall_t = 0.018
    top_bottom_t = 0.018
    enclosure.visual(
        Box((side_wall_t, LINER_DEPTH, OPENING_H)),
        origin=Origin(
            xyz=(OPENING_X - OPENING_W / 2.0 - side_wall_t / 2.0, wall_y, OPENING_Z)
        ),
        material=satin_graphite,
        name="liner_left",
    )
    enclosure.visual(
        Box((side_wall_t, LINER_DEPTH, OPENING_H)),
        origin=Origin(
            xyz=(OPENING_X + OPENING_W / 2.0 + side_wall_t / 2.0, wall_y, OPENING_Z)
        ),
        material=satin_graphite,
        name="liner_right",
    )
    enclosure.visual(
        Box((OPENING_W + 2.0 * side_wall_t, LINER_DEPTH, top_bottom_t)),
        origin=Origin(
            xyz=(OPENING_X, wall_y, OPENING_Z + OPENING_H / 2.0 + top_bottom_t / 2.0)
        ),
        material=satin_graphite,
        name="liner_top",
    )
    enclosure.visual(
        Box((OPENING_W + 2.0 * side_wall_t, LINER_DEPTH, top_bottom_t)),
        origin=Origin(
            xyz=(OPENING_X, wall_y, OPENING_Z - OPENING_H / 2.0 - top_bottom_t / 2.0)
        ),
        material=satin_graphite,
        name="liner_bottom",
    )
    enclosure.visual(
        Box((OPENING_W + 2.0 * side_wall_t, 0.004, OPENING_H + 2.0 * top_bottom_t)),
        origin=Origin(xyz=(OPENING_X, -(FACE_T / 2.0 + LINER_DEPTH) + 0.002, OPENING_Z)),
        material=shadow_black,
        name="service_bay_back",
    )
    enclosure.visual(
        Box((0.152, 0.022, 0.248)),
        origin=Origin(xyz=(OPENING_X + 0.060, -(FACE_T / 2.0 + LINER_DEPTH) + 0.013, 0.030)),
        material=satin_graphite,
        name="service_module",
    )
    enclosure.visual(
        Box((0.092, 0.010, 0.040)),
        origin=Origin(xyz=(OPENING_X - 0.078, -(FACE_T / 2.0 + LINER_DEPTH) + 0.007, -0.110)),
        material=rubber_black,
        name="cable_boot",
    )

    fastener_positions = [
        (OPENING_X - (OPENING_W / 2.0 + 0.030), OPENING_Z + (OPENING_H / 2.0 + 0.034)),
        (OPENING_X + (OPENING_W / 2.0 + 0.030), OPENING_Z + (OPENING_H / 2.0 + 0.034)),
        (OPENING_X - (OPENING_W / 2.0 + 0.030), OPENING_Z - (OPENING_H / 2.0 + 0.034)),
        (OPENING_X + (OPENING_W / 2.0 + 0.030), OPENING_Z - (OPENING_H / 2.0 + 0.034)),
    ]
    for idx, (x_pos, z_pos) in enumerate(fastener_positions, start=1):
        enclosure.visual(
            Cylinder(radius=0.007, length=0.0025),
            origin=Origin(
                xyz=(x_pos, FACE_T / 2.0 + 0.0015, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_steel,
            name=f"frame_fastener_{idx}",
        )

    receiver_x = OPENING_RIGHT + 0.012
    for name, z_pos in (("latch_receiver_upper", 0.150), ("latch_receiver_lower", -0.150)):
        enclosure.visual(
            Box((0.018, 0.005, 0.034)),
            origin=Origin(xyz=(receiver_x, -0.001, z_pos)),
            material=satin_titanium,
            name=name,
        )

    base_knuckle_specs = [
        ("base_knuckle_top", 0.231),
        ("base_knuckle_mid", 0.011),
        ("base_knuckle_bottom", -0.209),
    ]
    for name, z_pos in base_knuckle_specs:
        enclosure.visual(
            Cylinder(radius=0.006, length=0.110),
            origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, z_pos)),
            material=brushed_steel,
            name=name,
        )
        enclosure.visual(
            Box((0.008, 0.006, 0.060)),
            origin=Origin(xyz=(HINGE_AXIS_X - 0.010, HINGE_AXIS_Y, z_pos)),
            material=satin_graphite,
            name=f"{name}_strap",
        )

    enclosure.inertial = Inertial.from_geometry(
        Box((ENCLOSURE_W, 0.090, ENCLOSURE_H)),
        mass=8.4,
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        door_body_mesh,
        origin=Origin(xyz=(DOOR_CENTER_X, PANEL_BODY_Y, 0.0)),
        material=satin_graphite,
        name="panel_body",
    )
    access_panel.visual(
        door_trim_mesh,
        origin=Origin(xyz=(DOOR_CENTER_X, PANEL_TRIM_Y, 0.0)),
        material=satin_titanium,
        name="panel_trim",
    )
    access_panel.visual(
        door_insert_mesh,
        origin=Origin(xyz=(DOOR_CENTER_X, PANEL_INSERT_Y, 0.0)),
        material=matte_graphite,
        name="panel_insert",
    )

    for name, z_pos in (("panel_knuckle_upper", 0.121), ("panel_knuckle_lower", -0.099)):
        access_panel.visual(
            Cylinder(radius=0.006, length=0.110),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=brushed_steel,
            name=name,
        )
        access_panel.visual(
            Box((0.010, 0.006, 0.060)),
            origin=Origin(xyz=(0.007, 0.0, z_pos)),
            material=satin_graphite,
            name=f"{name}_strap",
        )

    latch_x = DOOR_LEFT_FROM_AXIS + DOOR_W - 0.018
    for name, z_pos in (("latch_upper", 0.150), ("latch_lower", -0.150)):
        access_panel.visual(
            Cylinder(radius=0.010, length=0.0030),
            origin=Origin(
                xyz=(latch_x, -0.0005, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_steel,
            name=name,
        )
        access_panel.visual(
            Box((0.012, 0.0012, 0.0035)),
            origin=Origin(xyz=(latch_x, 0.0010, z_pos)),
            material=shadow_black,
            name=f"{name}_slot",
        )

    access_panel.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        mass=2.2,
        origin=Origin(xyz=(DOOR_CENTER_X, PANEL_BODY_Y, 0.0)),
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent="enclosure",
        child="access_panel",
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_contact("access_panel", "enclosure")
    ctx.expect_aabb_gap(
        "enclosure",
        "access_panel",
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="service_frame",
        negative_elem="panel_trim",
        name="flush_service_frame_seam",
    )
    ctx.expect_joint_motion_axis(
        "panel_hinge",
        "access_panel",
        world_axis="y",
        direction="positive",
        min_delta=0.060,
    )
    ctx.expect_aabb_overlap("access_panel", "enclosure", axes="xz", min_overlap=0.250)
    ctx.expect_origin_distance("access_panel", "enclosure", axes="z", max_dist=0.030)
    ctx.expect_aabb_gap(
        "enclosure",
        "access_panel",
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="base_knuckle_top",
        negative_elem="panel_knuckle_upper",
        name="upper_hinge_knuckle_stack",
    )
    ctx.expect_aabb_gap(
        "access_panel",
        "enclosure",
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="panel_knuckle_upper",
        negative_elem="base_knuckle_mid",
        name="middle_hinge_knuckle_stack",
    )
    ctx.expect_aabb_gap(
        "access_panel",
        "enclosure",
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="panel_knuckle_lower",
        negative_elem="base_knuckle_bottom",
        name="lower_hinge_knuckle_stack",
    )
    ctx.expect_aabb_gap(
        "enclosure",
        "access_panel",
        axis="x",
        max_gap=0.020,
        max_penetration=0.0,
        positive_elem="latch_receiver_upper",
        negative_elem="latch_upper",
        name="upper_latch_alignment",
    )
    ctx.expect_aabb_gap(
        "enclosure",
        "access_panel",
        axis="x",
        max_gap=0.020,
        max_penetration=0.0,
        positive_elem="latch_receiver_lower",
        negative_elem="latch_lower",
        name="lower_latch_alignment",
    )
    with ctx.pose({"panel_hinge": 1.15}):
        ctx.expect_aabb_overlap("access_panel", "enclosure", axes="z", min_overlap=0.450)
        ctx.expect_origin_distance("access_panel", "enclosure", axes="z", max_dist=0.040)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
