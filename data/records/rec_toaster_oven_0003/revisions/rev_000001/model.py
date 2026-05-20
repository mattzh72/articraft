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

BODY_W = 0.480
BODY_D = 0.400
BODY_H = 0.285
FOOT_H = 0.010
SHELL_T = 0.010
FRONT_FRAME_T = 0.014

DOOR_CENTER_X = -0.051
DOOR_W = 0.316
DOOR_H = 0.176
DOOR_T = 0.024
DOOR_HINGE_Y = -0.212
DOOR_HINGE_Z = 0.066

CAVITY_CENTER_X = DOOR_CENTER_X
CAVITY_Y = -0.030
CAVITY_W_OUTER = 0.320
CAVITY_W_INNER = 0.312
CAVITY_D = 0.308
CAVITY_BOTTOM_Z = 0.066
CAVITY_TOP_Z = 0.234
LINER_T = 0.004


def _add_knob(
    model: ArticulatedObject,
    parent,
    *,
    knob_name: str,
    x: float,
    z: float,
    knob_material,
    accent_material,
) -> None:
    knob = model.part(knob_name)
    knob.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="body",
    )
    knob.visual(
        Cylinder(radius=0.0125, length=0.006),
        origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_material,
        name="cap",
    )
    knob.visual(
        Box((0.002, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, -0.0185, 0.015,)),
        material=accent_material,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.042, 0.028, 0.042)),
        mass=0.07,
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
    )
    model.articulation(
        f"fascia_to_{knob_name}",
        ArticulationType.REVOLUTE,
        parent=parent,
        child=knob,
        origin=Origin(xyz=(x, -0.208, z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=3.5,
            lower=-2.35,
            upper=2.35,
        ),
    )


def _add_foot(
    model: ArticulatedObject,
    parent,
    *,
    foot_name: str,
    x: float,
    y: float,
    material,
) -> None:
    foot = model.part(foot_name)
    foot.visual(
        Box((0.050, 0.030, FOOT_H)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H * 0.5)),
        material=material,
        name="foot_pad",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.050, 0.030, FOOT_H)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, FOOT_H * 0.5)),
    )
    model.articulation(
        f"carcass_to_{foot_name}",
        ArticulationType.FIXED,
        parent=parent,
        child=foot,
        origin=Origin(xyz=(x, y, 0.0)),
    )


def _translate_profile(profile, dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _make_rounded_panel_mesh(
    mesh_name: str,
    *,
    width: float,
    height: float,
    depth: float,
    radius: float,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=6),
        depth,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(mesh_name))


def _make_top_vent_mesh(mesh_name: str):
    outer_profile = rounded_rect_profile(0.220, 0.066, 0.008, corner_segments=6)
    slot_profile = rounded_rect_profile(0.024, 0.005, 0.0018, corner_segments=4)
    hole_profiles = []
    for row_y in (-0.013, 0.013):
        for slot_x in (-0.072, -0.036, 0.0, 0.036, 0.072):
            hole_profiles.append(_translate_profile(slot_profile, slot_x, row_y))
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        0.0015,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(mesh_name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_toaster_oven", assets=ASSETS)

    body_matte = model.material("body_matte_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    fascia_satin = model.material("fascia_satin_stainless", rgba=(0.72, 0.73, 0.75, 1.0))
    trim_satin = model.material("trim_dark_satin", rgba=(0.28, 0.29, 0.31, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    liner_steel = model.material("liner_steel", rgba=(0.77, 0.78, 0.80, 1.0))
    rack_steel = model.material("rack_steel", rgba=(0.70, 0.71, 0.73, 1.0))
    tray_steel = model.material("tray_steel", rgba=(0.64, 0.65, 0.67, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.74, 0.75, 0.78, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.16, 0.19, 0.22, 0.38))
    rubber = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    fascia_panel_mesh = _make_rounded_panel_mesh(
        "premium_toaster_oven_fascia.obj",
        width=0.104,
        height=0.235,
        depth=0.006,
        radius=0.012,
    )
    top_vent_mesh = _make_top_vent_mesh("premium_toaster_oven_top_vent.obj")

    carcass = model.part("carcass")
    side_shell_height = BODY_H - FOOT_H - 2.0 * SHELL_T
    side_shell_center_z = FOOT_H + SHELL_T + side_shell_height * 0.5

    carcass.visual(
        Box((BODY_W, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + SHELL_T * 0.5)),
        material=body_matte,
        name="base_shell",
    )
    carcass.visual(
        Box((BODY_W, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - SHELL_T * 0.5)),
        material=body_matte,
        name="top_shell",
    )
    carcass.visual(
        Box((SHELL_T, BODY_D, side_shell_height)),
        origin=Origin(xyz=(-BODY_W * 0.5 + SHELL_T * 0.5, 0.0, side_shell_center_z)),
        material=body_matte,
        name="left_shell",
    )
    carcass.visual(
        Box((SHELL_T, BODY_D, side_shell_height)),
        origin=Origin(xyz=(BODY_W * 0.5 - SHELL_T * 0.5, 0.0, side_shell_center_z)),
        material=body_matte,
        name="right_shell",
    )
    carcass.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, side_shell_height)),
        origin=Origin(
            xyz=(0.0, BODY_D * 0.5 - SHELL_T * 0.5, side_shell_center_z),
        ),
        material=body_matte,
        name="rear_shell",
    )
    carcass.visual(
        Box((0.026, FRONT_FRAME_T, 0.205)),
        origin=Origin(xyz=(-0.227, -0.193, 0.1525)),
        material=body_matte,
        name="left_jamb",
    )
    carcass.visual(
        Box((0.016, FRONT_FRAME_T, 0.205)),
        origin=Origin(xyz=(0.120, -0.193, 0.1525)),
        material=body_matte,
        name="right_door_partition",
    )
    carcass.visual(
        Box((0.360, FRONT_FRAME_T, 0.030)),
        origin=Origin(xyz=(-0.040, -0.193, 0.035)),
        material=body_matte,
        name="door_sill",
    )
    carcass.visual(
        Box((BODY_W, FRONT_FRAME_T, 0.020)),
        origin=Origin(xyz=(0.0, -0.193, 0.265)),
        material=body_matte,
        name="top_front_band",
    )
    carcass.visual(
        Box((0.098, 0.040, 0.205)),
        origin=Origin(xyz=(0.181, -0.180, 0.1525)),
        material=body_matte,
        name="control_spine",
    )
    carcass.visual(
        Box((0.098, 0.040, 0.030)),
        origin=Origin(xyz=(0.181, -0.180, 0.035)),
        material=body_matte,
        name="control_pedestal",
    )
    carcass.visual(
        Box((0.006, 0.026, 0.020)),
        origin=Origin(xyz=(-0.213, -0.213, 0.060)),
        material=trim_satin,
        name="left_hinge_block",
    )
    carcass.visual(
        Box((0.006, 0.026, 0.020)),
        origin=Origin(xyz=(0.111, -0.213, 0.060)),
        material=trim_satin,
        name="right_hinge_block",
    )
    carcass.visual(
        top_vent_mesh,
        origin=Origin(xyz=(-0.020, 0.092, BODY_H - 0.00025)),
        material=trim_satin,
        name="top_vent_grille",
    )
    carcass.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    liner = model.part("liner")
    liner_center_z = (CAVITY_BOTTOM_Z + CAVITY_TOP_Z) * 0.5
    liner_height = CAVITY_TOP_Z - CAVITY_BOTTOM_Z - 2.0 * LINER_T
    liner.visual(
        Box((CAVITY_W_OUTER, CAVITY_D, LINER_T)),
        origin=Origin(xyz=(CAVITY_CENTER_X, CAVITY_Y, CAVITY_BOTTOM_Z + LINER_T * 0.5)),
        material=liner_steel,
        name="floor",
    )
    liner.visual(
        Box((CAVITY_W_OUTER, CAVITY_D, LINER_T)),
        origin=Origin(xyz=(CAVITY_CENTER_X, CAVITY_Y, CAVITY_TOP_Z - LINER_T * 0.5)),
        material=liner_steel,
        name="ceiling",
    )
    liner.visual(
        Box((LINER_T, CAVITY_D, liner_height)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X - CAVITY_W_OUTER * 0.5 + LINER_T * 0.5, CAVITY_Y, liner_center_z),
        ),
        material=liner_steel,
        name="left_wall",
    )
    liner.visual(
        Box((LINER_T, CAVITY_D, liner_height)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X + CAVITY_W_OUTER * 0.5 - LINER_T * 0.5, CAVITY_Y, liner_center_z),
        ),
        material=liner_steel,
        name="right_wall",
    )
    liner.visual(
        Box((CAVITY_W_OUTER, LINER_T, CAVITY_TOP_Z - CAVITY_BOTTOM_Z)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X, CAVITY_Y + CAVITY_D * 0.5 - LINER_T * 0.5, liner_center_z),
        ),
        material=liner_steel,
        name="back_wall",
    )
    liner.visual(
        Box((LINER_T, LINER_T, CAVITY_TOP_Z - CAVITY_BOTTOM_Z)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X - CAVITY_W_OUTER * 0.5 + LINER_T * 0.5, -0.184, liner_center_z),
        ),
        material=liner_steel,
        name="front_left_flange",
    )
    liner.visual(
        Box((LINER_T, LINER_T, CAVITY_TOP_Z - CAVITY_BOTTOM_Z)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X + CAVITY_W_OUTER * 0.5 - LINER_T * 0.5, -0.184, liner_center_z),
        ),
        material=liner_steel,
        name="front_right_flange",
    )
    liner.visual(
        Box((CAVITY_W_OUTER, LINER_T, LINER_T)),
        origin=Origin(xyz=(CAVITY_CENTER_X, -0.184, CAVITY_TOP_Z - LINER_T * 0.5)),
        material=liner_steel,
        name="front_top_flange",
    )
    liner.visual(
        Box((CAVITY_W_OUTER, LINER_T, LINER_T)),
        origin=Origin(xyz=(CAVITY_CENTER_X, -0.184, CAVITY_BOTTOM_Z + LINER_T * 0.5)),
        material=liner_steel,
        name="front_bottom_flange",
    )
    liner.visual(
        Box((0.018, 0.250, 0.008)),
        origin=Origin(xyz=(CAVITY_CENTER_X - 0.147, -0.040, 0.100)),
        material=liner_steel,
        name="left_rack_guide",
    )
    liner.visual(
        Box((0.018, 0.250, 0.008)),
        origin=Origin(xyz=(CAVITY_CENTER_X + 0.147, -0.040, 0.100)),
        material=liner_steel,
        name="right_rack_guide",
    )
    liner.visual(
        Box((0.008, 0.260, 0.010)),
        origin=Origin(xyz=(CAVITY_CENTER_X - 0.147, -0.060, 0.061)),
        material=liner_steel,
        name="left_tray_guide",
    )
    liner.visual(
        Box((0.008, 0.260, 0.010)),
        origin=Origin(xyz=(CAVITY_CENTER_X + 0.147, -0.060, 0.061)),
        material=liner_steel,
        name="right_tray_guide",
    )
    liner.visual(
        Box((0.010, LINER_T, 0.024)),
        origin=Origin(xyz=(-0.213, -0.184, 0.098)),
        material=liner_steel,
        name="left_mount_tab",
    )
    liner.visual(
        Box((0.010, LINER_T, 0.024)),
        origin=Origin(xyz=(0.111, -0.184, 0.098)),
        material=liner_steel,
        name="right_mount_tab",
    )
    liner.inertial = Inertial.from_geometry(
        Box((CAVITY_W_OUTER, CAVITY_D, CAVITY_TOP_Z - CAVITY_BOTTOM_Z)),
        mass=2.2,
        origin=Origin(xyz=(CAVITY_CENTER_X, CAVITY_Y, liner_center_z)),
    )
    model.articulation(
        "carcass_to_liner",
        ArticulationType.FIXED,
        parent=carcass,
        child=liner,
        origin=Origin(),
    )

    fascia = model.part("fascia")
    fascia.visual(
        fascia_panel_mesh,
        origin=Origin(xyz=(0.182, -0.203, 0.1575), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fascia_satin,
        name="fascia_skin",
    )
    fascia.visual(
        Box((0.084, 0.003, 0.016)),
        origin=Origin(xyz=(0.182, -0.2065, 0.048)),
        material=trim_satin,
        name="lower_trim_band",
    )
    fascia.visual(
        Box((0.060, 0.002, 0.010)),
        origin=Origin(xyz=(0.182, -0.207, 0.247)),
        material=trim_satin,
        name="status_window",
    )
    for index, knob_z in enumerate((0.208, 0.158, 0.108)):
        fascia.visual(
            Cylinder(radius=0.024, length=0.0025),
            origin=Origin(
                xyz=(0.182, -0.20725, knob_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim_satin,
            name=f"bezel_{index}",
        )
    fascia.inertial = Inertial.from_geometry(
        Box((0.104, 0.006, 0.235)),
        mass=0.45,
        origin=Origin(xyz=(0.182, -0.203, 0.1575)),
    )
    model.articulation(
        "carcass_to_fascia",
        ArticulationType.FIXED,
        parent=carcass,
        child=fascia,
        origin=Origin(),
    )

    door = model.part("door")
    door.visual(
        Box((0.020, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-DOOR_W * 0.5 + 0.010, 0.0, DOOR_H * 0.5)),
        material=trim_satin,
        name="left_frame",
    )
    door.visual(
        Box((0.020, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_W * 0.5 - 0.010, 0.0, DOOR_H * 0.5)),
        material=trim_satin,
        name="right_frame",
    )
    door.visual(
        Box((DOOR_W, DOOR_T, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, DOOR_H - 0.010)),
        material=trim_satin,
        name="top_frame",
    )
    door.visual(
        Box((DOOR_W, DOOR_T, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim_satin,
        name="bottom_frame",
    )
    door.visual(
        Box((0.248, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, 0.006, 0.037)),
        material=trim_satin,
        name="lower_inner_trim",
    )
    door.visual(
        Box((0.244, 0.004, 0.100)),
        origin=Origin(xyz=(0.0, -0.006, 0.106)),
        material=glass_smoke,
        name="window_glass",
    )
    door.visual(
        Box((0.016, 0.016, 0.012)),
        origin=Origin(xyz=(-0.151, -0.008, 0.006)),
        material=trim_satin,
        name="left_hinge_knuckle",
    )
    door.visual(
        Box((0.016, 0.016, 0.012)),
        origin=Origin(xyz=(0.151, -0.008, 0.006)),
        material=trim_satin,
        name="right_hinge_knuckle",
    )
    door.visual(
        Box((0.022, 0.008, 0.024)),
        origin=Origin(xyz=(-0.072, -0.008, 0.122)),
        material=trim_satin,
        name="left_handle_boss",
    )
    door.visual(
        Box((0.022, 0.008, 0.024)),
        origin=Origin(xyz=(0.072, -0.008, 0.122)),
        material=trim_satin,
        name="right_handle_boss",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, DOOR_H * 0.5)),
    )
    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.4,
            lower=0.0,
            upper=1.42,
        ),
    )

    handle = model.part("door_handle")
    handle.visual(
        Cylinder(radius=0.008, length=0.182),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="grip",
    )
    handle.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(-0.072, 0.001, 0.0)),
        material=handle_metal,
        name="left_post",
    )
    handle.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(0.072, 0.001, 0.0)),
        material=handle_metal,
        name="right_post",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.190, 0.020, 0.020)),
        mass=0.18,
        origin=Origin(),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.FIXED,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.0, -0.022, 0.122)),
    )

    rack = model.part("rack")
    rack.visual(
        Box((0.006, 0.220, 0.020)),
        origin=Origin(xyz=(-0.145, 0.0, 0.010)),
        material=rack_steel,
        name="left_side_frame",
    )
    rack.visual(
        Box((0.006, 0.220, 0.020)),
        origin=Origin(xyz=(0.145, 0.0, 0.010)),
        material=rack_steel,
        name="right_side_frame",
    )
    rack.visual(
        Box((0.292, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.107, 0.020)),
        material=rack_steel,
        name="front_rail",
    )
    rack.visual(
        Box((0.292, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.107, 0.020)),
        material=rack_steel,
        name="rear_rail",
    )
    for index, slat_y in enumerate((-0.070, -0.035, 0.000, 0.035, 0.070)):
        rack.visual(
            Box((0.284, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, slat_y, 0.020)),
            material=rack_steel,
            name=f"slat_{index}",
        )
    rack.inertial = Inertial.from_geometry(
        Box((0.292, 0.220, 0.020)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    model.articulation(
        "liner_to_rack",
        ArticulationType.PRISMATIC,
        parent=liner,
        child=rack,
        origin=Origin(xyz=(CAVITY_CENTER_X, -0.040, 0.104)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.18,
            lower=0.0,
            upper=0.110,
        ),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((0.286, 0.240, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=tray_steel,
        name="tray_base",
    )
    crumb_tray.visual(
        Box((0.008, 0.240, 0.004)),
        origin=Origin(xyz=(-0.147, 0.0, 0.006)),
        material=tray_steel,
        name="left_lip",
    )
    crumb_tray.visual(
        Box((0.008, 0.240, 0.004)),
        origin=Origin(xyz=(0.147, 0.0, 0.006)),
        material=tray_steel,
        name="right_lip",
    )
    crumb_tray.visual(
        Box((0.286, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.117, 0.009)),
        material=tray_steel,
        name="rear_lip",
    )
    crumb_tray.visual(
        Box((0.290, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.127, 0.013)),
        material=trim_satin,
        name="front_pull",
    )
    crumb_tray.inertial = Inertial.from_geometry(
        Box((0.290, 0.254, 0.020)),
        mass=0.36,
        origin=Origin(xyz=(0.0, -0.005, 0.010)),
    )
    model.articulation(
        "liner_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=liner,
        child=crumb_tray,
        origin=Origin(xyz=(CAVITY_CENTER_X, -0.063, 0.048)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.18,
            lower=0.0,
            upper=0.115,
        ),
    )

    _add_knob(
        model,
        fascia,
        knob_name="function_knob",
        x=0.182,
        z=0.208,
        knob_material=knob_dark,
        accent_material=fascia_satin,
    )
    _add_knob(
        model,
        fascia,
        knob_name="temperature_knob",
        x=0.182,
        z=0.158,
        knob_material=knob_dark,
        accent_material=fascia_satin,
    )
    _add_knob(
        model,
        fascia,
        knob_name="timer_knob",
        x=0.182,
        z=0.108,
        knob_material=knob_dark,
        accent_material=fascia_satin,
    )

    _add_foot(model, carcass, foot_name="front_left_foot", x=-0.160, y=-0.145, material=rubber)
    _add_foot(model, carcass, foot_name="front_right_foot", x=0.160, y=-0.145, material=rubber)
    _add_foot(model, carcass, foot_name="rear_left_foot", x=-0.160, y=0.145, material=rubber)
    _add_foot(model, carcass, foot_name="rear_right_foot", x=0.160, y=0.145, material=rubber)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    carcass = object_model.get_part("carcass")
    liner = object_model.get_part("liner")
    fascia = object_model.get_part("fascia")
    door = object_model.get_part("door")
    handle = object_model.get_part("door_handle")
    rack = object_model.get_part("rack")
    crumb_tray = object_model.get_part("crumb_tray")
    function_knob = object_model.get_part("function_knob")
    temperature_knob = object_model.get_part("temperature_knob")
    timer_knob = object_model.get_part("timer_knob")
    feet = [
        object_model.get_part("front_left_foot"),
        object_model.get_part("front_right_foot"),
        object_model.get_part("rear_left_foot"),
        object_model.get_part("rear_right_foot"),
    ]

    door_hinge = object_model.get_articulation("carcass_to_door")
    rack_slide = object_model.get_articulation("liner_to_rack")
    tray_slide = object_model.get_articulation("liner_to_crumb_tray")
    knob_joints = [
        object_model.get_articulation("fascia_to_function_knob"),
        object_model.get_articulation("fascia_to_temperature_knob"),
        object_model.get_articulation("fascia_to_timer_knob"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(liner, carcass, name="liner_mounted_to_carcass")
    ctx.expect_contact(fascia, carcass, name="fascia_mounted_to_carcass")
    ctx.expect_contact(handle, door, name="handle_mounted_to_door")
    ctx.expect_contact(rack, liner, name="rack_supported_on_guides")
    ctx.expect_contact(crumb_tray, liner, name="crumb_tray_supported_on_guides")

    for foot in feet:
        ctx.expect_contact(foot, carcass, name=f"{foot.name}_mounted")

    for knob in (function_knob, temperature_knob, timer_knob):
        ctx.expect_contact(knob, fascia, name=f"{knob.name}_mounted")

    ctx.expect_origin_gap(
        carcass,
        door,
        axis="y",
        min_gap=0.19,
        max_gap=0.23,
        name="door_at_front_face",
    )
    ctx.expect_overlap(door, carcass, axes="xz", min_overlap=0.17, name="door_covers_opening")
    ctx.expect_gap(rack, door, axis="y", min_gap=0.045, name="rack_clear_of_closed_door")
    ctx.expect_within(rack, liner, axes="x", margin=0.01, name="rack_centered_in_cavity")
    ctx.expect_within(crumb_tray, liner, axes="x", margin=0.02, name="tray_centered_below_cavity")

    ctx.check(
        "door_hinge_axis",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        f"unexpected door hinge axis: {door_hinge.axis}",
    )
    ctx.check(
        "rack_slide_axis",
        tuple(rack_slide.axis) == (0.0, -1.0, 0.0),
        f"unexpected rack slide axis: {rack_slide.axis}",
    )
    ctx.check(
        "tray_slide_axis",
        tuple(tray_slide.axis) == (0.0, -1.0, 0.0),
        f"unexpected tray slide axis: {tray_slide.axis}",
    )
    for knob_joint in knob_joints:
        ctx.check(
            f"{knob_joint.name}_axis",
            tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
            f"unexpected knob axis: {knob_joint.axis}",
        )

    door_closed_aabb = ctx.part_world_aabb(door)
    if door_closed_aabb is None:
        ctx.fail("door_closed_aabb_present", "door AABB unavailable in rest pose")
    else:
        with ctx.pose({door_hinge: 1.35}):
            door_open_aabb = ctx.part_world_aabb(door)
            if door_open_aabb is None:
                ctx.fail("door_open_aabb_present", "door AABB unavailable in open pose")
            else:
                ctx.check(
                    "door_opens_downward",
                    door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.09
                    and door_open_aabb[0][1] < door_closed_aabb[0][1] - 0.10,
                    f"closed={door_closed_aabb}, open={door_open_aabb}",
                )
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="door_open_no_floating")

    rack_rest_pos = ctx.part_world_position(rack)
    if rack_rest_pos is None:
        ctx.fail("rack_rest_position_present", "rack origin position unavailable")
    else:
        with ctx.pose({rack_slide: 0.110}):
            rack_open_pos = ctx.part_world_position(rack)
            if rack_open_pos is None:
                ctx.fail("rack_open_position_present", "rack extended position unavailable")
            else:
                ctx.check(
                    "rack_slides_forward",
                    rack_open_pos[1] < rack_rest_pos[1] - 0.10,
                    f"rest={rack_rest_pos}, extended={rack_open_pos}",
                )

    tray_rest_pos = ctx.part_world_position(crumb_tray)
    if tray_rest_pos is None:
        ctx.fail("tray_rest_position_present", "crumb tray origin position unavailable")
    else:
        with ctx.pose({tray_slide: 0.115}):
            tray_open_pos = ctx.part_world_position(crumb_tray)
            if tray_open_pos is None:
                ctx.fail("tray_open_position_present", "crumb tray extended position unavailable")
            else:
                ctx.check(
                    "crumb_tray_slides_forward",
                    tray_open_pos[1] < tray_rest_pos[1] - 0.105,
                    f"rest={tray_rest_pos}, extended={tray_open_pos}",
                )

    for knob_part, knob_joint in (
        (function_knob, knob_joints[0]),
        (temperature_knob, knob_joints[1]),
        (timer_knob, knob_joints[2]),
    ):
        rest_indicator_aabb = ctx.part_element_world_aabb(knob_part, elem="indicator")
        if rest_indicator_aabb is None:
            ctx.fail(f"{knob_part.name}_indicator_present", "indicator AABB unavailable at rest")
            continue
        with ctx.pose({knob_joint: 1.6}):
            turned_indicator_aabb = ctx.part_element_world_aabb(knob_part, elem="indicator")
            if turned_indicator_aabb is None:
                ctx.fail(
                    f"{knob_part.name}_indicator_turned_present",
                    "indicator AABB unavailable when turned",
                )
            else:
                ctx.check(
                    f"{knob_part.name}_turns",
                    abs(turned_indicator_aabb[0][0] - rest_indicator_aabb[0][0]) > 0.008
                    or abs(turned_indicator_aabb[1][2] - rest_indicator_aabb[1][2]) > 0.008,
                    f"rest={rest_indicator_aabb}, turned={turned_indicator_aabb}",
                )

    with ctx.pose({door_hinge: 1.35, rack_slide: 0.110}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_rack_extended_no_overlap")
    with ctx.pose({door_hinge: 1.35, tray_slide: 0.115}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_tray_extended_no_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
