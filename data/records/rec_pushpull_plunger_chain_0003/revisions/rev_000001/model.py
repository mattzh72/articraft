from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

AXIS_Z = 0.100
UPPER_ROD_Z = 0.158
LOWER_ROD_Z = 0.042
GUIDE_Y = 0.056
PRIMARY_TRAVEL = 0.080
SECONDARY_TRAVEL = 0.095


def _x_cylinder(
    radius: float,
    x0: float,
    x1: float,
    z: float = AXIS_Z,
    y: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(x1 - x0).translate((x0, y, z))


def _x_tube(
    outer_radius: float,
    inner_radius: float,
    x0: float,
    x1: float,
    z: float = AXIS_Z,
    y: float = 0.0,
) -> cq.Workplane:
    outer = _x_cylinder(outer_radius, x0, x1, z, y=y)
    inner = _x_cylinder(inner_radius, x0 - 0.001, x1 + 0.001, z, y=y)
    return outer.cut(inner)


def _x_hex(flat_diameter: float, x0: float, x1: float, z: float, y: float = 0.0) -> cq.Workplane:
    return cq.Workplane("YZ").polygon(6, flat_diameter).extrude(x1 - x0).translate((x0, y, z))


def _y_cylinder(radius: float, y0: float, y1: float, x: float, z: float = AXIS_Z) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(y1 - y0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0)
        .translate((x, y0, z))
    )


def _z_cylinder(radius: float, z0: float, z1: float, x: float, y: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(z1 - z0).translate((x, y, z0))


def _z_hex(flat_diameter: float, z0: float, z1: float, x: float, y: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").polygon(6, flat_diameter).extrude(z1 - z0).translate((x, y, z0))


def _box(
    length: float,
    width: float,
    height: float,
    *,
    x0: float,
    y: float = 0.0,
    z0: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, False))
        .translate((x0, y, z0))
    )


def _gusset(points_xz: list[tuple[float, float]], y0: float, thickness: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points_xz).close().extrude(thickness).translate((0.0, y0, 0.0))


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=0.0008,
        angular_tolerance=0.08,
    )


def _make_support_frame_visuals() -> tuple[cq.Workplane, cq.Workplane]:
    base = _box(0.68, 0.18, 0.014, x0=-0.32, z0=0.0)
    for foot_x in (-0.24, 0.22):
        for foot_y in (-0.055, 0.055):
            base = base.union(_box(0.050, 0.038, 0.012, x0=foot_x, y=foot_y, z0=-0.012))
    for slot_x in (-0.226, 0.234):
        for slot_y in (-0.055, 0.055):
            base = base.cut(_box(0.022, 0.010, 0.030, x0=slot_x, y=slot_y, z0=-0.015))

    rear_plate = _box(0.018, 0.132, 0.154, x0=-0.216, z0=0.023)
    rear_plate = rear_plate.cut(_x_cylinder(0.0368, -0.218, -0.196))
    rear_plate = rear_plate.cut(_x_cylinder(0.0110, -0.218, -0.196, UPPER_ROD_Z))
    rear_plate = rear_plate.cut(_x_cylinder(0.0110, -0.218, -0.196, LOWER_ROD_Z))
    for bolt_y in (-0.034, 0.034):
        for bolt_z in (AXIS_Z - 0.034, AXIS_Z + 0.034):
            rear_plate = rear_plate.cut(_x_cylinder(0.0052, -0.218, -0.196, bolt_z, y=bolt_y))

    rear_gusset_pos = _gusset([(-0.262, 0.014), (-0.216, 0.014), (-0.216, 0.116)], 0.046, 0.014)
    rear_gusset_neg = _gusset([(-0.262, 0.014), (-0.216, 0.014), (-0.216, 0.116)], -0.060, 0.014)
    rear_frame = base.union(rear_plate).union(rear_gusset_pos).union(rear_gusset_neg)

    front_yoke = _box(0.018, 0.112, 0.154, x0=0.244, z0=0.023)
    front_yoke = front_yoke.cut(_x_cylinder(0.0110, 0.242, 0.264, UPPER_ROD_Z))
    front_yoke = front_yoke.cut(_x_cylinder(0.0110, 0.242, 0.264, LOWER_ROD_Z))
    front_yoke = front_yoke.cut(_box(0.024, 0.064, 0.064, x0=0.238, z0=AXIS_Z - 0.032))
    front_gusset_pos = _gusset([(0.208, 0.014), (0.244, 0.014), (0.244, 0.108)], 0.042, 0.014)
    front_gusset_neg = _gusset([(0.208, 0.014), (0.244, 0.014), (0.244, 0.108)], -0.056, 0.014)
    front_yoke = front_yoke.union(front_gusset_pos).union(front_gusset_neg)

    return rear_frame, front_yoke


def _make_outer_sleeve_visuals() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane, cq.Workplane, cq.Workplane]:
    body_shell = _box(0.308, 0.094, 0.176, x0=-0.150, z0=0.012)
    body_shell = body_shell.cut(_box(0.284, 0.070, 0.152, x0=-0.138, z0=0.024))
    side_pad = _box(0.162, 0.018, 0.144, x0=-0.020, y=0.038, z0=0.020)
    opening = _box(0.132, 0.030, 0.124, x0=-0.004, y=0.034, z0=0.030)
    body_shell = body_shell.union(side_pad).cut(opening)
    body_shell = body_shell.cut(_x_cylinder(0.0285, -0.180, 0.172))
    body_shell = body_shell.cut(_x_cylinder(0.0110, -0.180, 0.172, UPPER_ROD_Z))
    body_shell = body_shell.cut(_x_cylinder(0.0110, -0.180, 0.172, LOWER_ROD_Z))

    rear_flange = _box(0.024, 0.126, 0.126, x0=-0.184, z0=AXIS_Z - 0.063)
    rear_flange = rear_flange.cut(_x_cylinder(0.0290, -0.186, -0.158))
    rear_flange = rear_flange.cut(_x_cylinder(0.0110, -0.186, -0.158, UPPER_ROD_Z))
    rear_flange = rear_flange.cut(_x_cylinder(0.0110, -0.186, -0.158, LOWER_ROD_Z))
    for bolt_y in (-0.040, 0.040):
        for bolt_z in (AXIS_Z - 0.032, AXIS_Z + 0.032):
            rear_flange = rear_flange.cut(_x_cylinder(0.0052, -0.186, -0.158, bolt_z, y=bolt_y))

    gland_retainer = _x_tube(0.0310, 0.0226, 0.160, 0.174)
    gland_nut = _x_hex(0.094, 0.174, 0.196, AXIS_Z)
    gland_nut = gland_nut.cut(_x_cylinder(0.0226, 0.172, 0.198))
    wiper_seal = _x_tube(0.0234, 0.0208, 0.188, 0.196)
    return body_shell, rear_flange, gland_retainer, gland_nut, wiper_seal


def _make_access_cover() -> cq.Workplane:
    cover = _box(0.162, 0.008, 0.144, x0=-0.020, y=0.051, z0=0.020)
    for bolt_x in (-0.004, 0.058, 0.120):
        for bolt_z in (0.036, AXIS_Z, 0.164):
            cover = cover.union(_y_cylinder(0.0026, 0.043, 0.051, x=bolt_x, z=bolt_z))
            cover = cover.union(_y_cylinder(0.0050, 0.051, 0.057, x=bolt_x, z=bolt_z))
    return cover


def _make_guide_rod(z_level: float) -> cq.Workplane:
    rod = _x_cylinder(0.0080, -0.198, 0.244, z_level)
    rod = rod.union(_x_cylinder(0.0102, -0.216, -0.198, z_level))
    rod = rod.union(_x_cylinder(0.0102, 0.244, 0.262, z_level))
    rod = rod.union(_x_hex(0.022, -0.230, -0.216, z_level))
    rod = rod.union(_x_hex(0.022, 0.262, 0.276, z_level))
    return rod


def _make_primary_plunger_visuals() -> tuple[
    cq.Workplane,
    cq.Workplane,
    cq.Workplane,
    cq.Workplane,
    cq.Workplane,
    cq.Workplane,
    cq.Workplane,
]:
    central_sleeve = _x_tube(0.0215, 0.0162, -0.054, 0.186)
    central_sleeve = central_sleeve.union(_x_tube(0.0235, 0.0162, -0.084, -0.054))
    upper_bushing = _x_tube(0.0136, 0.0104, 0.010, 0.126, UPPER_ROD_Z)
    lower_bushing = _x_tube(0.0136, 0.0104, 0.010, 0.126, LOWER_ROD_Z)
    web_pos = _box(0.090, 0.014, UPPER_ROD_Z - LOWER_ROD_Z - 0.024, x0=0.024, y=0.020, z0=LOWER_ROD_Z + 0.012)
    web_neg = _box(0.090, 0.014, UPPER_ROD_Z - LOWER_ROD_Z - 0.024, x0=0.024, y=-0.020, z0=LOWER_ROD_Z + 0.012)
    rib_upper = _box(0.096, 0.042, 0.010, x0=0.016, z0=UPPER_ROD_Z - 0.005)
    rib_lower = _box(0.096, 0.042, 0.010, x0=0.016, z0=LOWER_ROD_Z - 0.005)
    bridge_webs = web_pos.union(web_neg).union(rib_upper).union(rib_lower)
    nose_bearing = _x_tube(0.0180, 0.0110, 0.170, 0.184)
    rod_seal = _x_tube(0.0130, 0.0108, 0.184, 0.188)
    stop_collar = _x_tube(0.0300, 0.0162, 0.199, 0.211)
    return central_sleeve, upper_bushing, lower_bushing, bridge_webs, nose_bearing, rod_seal, stop_collar


def _make_secondary_plunger_visuals() -> tuple[cq.Workplane, cq.Workplane]:
    rod_body = _x_cylinder(0.0152, -0.060, 0.162)
    rod_body = rod_body.union(_x_cylinder(0.0105, 0.162, 0.320))
    stop_collar = _x_tube(0.0230, 0.0105, 0.218, 0.232)
    return rod_body, stop_collar


def _make_terminal_clevis() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    shank = _x_cylinder(0.0140, 0.320, 0.344)
    body = _box(0.040, 0.040, 0.056, x0=0.332, z0=0.072)
    clevis = shank.union(body)
    clevis = clevis.cut(_box(0.028, 0.022, 0.036, x0=0.344, z0=0.082))
    pin_hole = cq.Workplane("XZ").circle(0.0050).extrude(0.038).translate((0.356, -0.019, AXIS_Z))
    clevis_body = clevis.cut(pin_hole)
    jam_nut = _x_hex(0.030, 0.306, 0.320, AXIS_Z).cut(_x_cylinder(0.0110, 0.304, 0.322))
    clevis_pin = _y_cylinder(0.0050, -0.019, 0.019, x=0.356)
    return clevis_body, jam_nut, clevis_pin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_pull_plunger_chain", assets=ASSETS)

    frame_mat = model.material("frame_powdercoat", rgba=(0.20, 0.23, 0.26, 1.0))
    steel_mat = model.material("machined_steel", rgba=(0.63, 0.65, 0.69, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    bronze_mat = model.material("bronze_bushing", rgba=(0.69, 0.53, 0.28, 1.0))

    rod_y = 0.065

    base = _box(0.68, 0.18, 0.014, x0=-0.32, z0=0.0)
    for foot_x in (-0.24, 0.22):
        for foot_y in (-0.055, 0.055):
            base = base.union(_box(0.050, 0.038, 0.012, x0=foot_x, y=foot_y, z0=-0.012))
    for slot_x in (-0.226, 0.234):
        for slot_y in (-0.055, 0.055):
            base = base.cut(_box(0.022, 0.010, 0.030, x0=slot_x, y=slot_y, z0=-0.015))

    rear_bracket = _box(0.014, 0.132, 0.154, x0=-0.206, z0=0.023)
    rear_bracket = rear_bracket.cut(_x_cylinder(0.0368, -0.208, -0.190))
    for y_val in (-rod_y, rod_y):
        rear_bracket = rear_bracket.cut(_x_cylinder(0.0090, -0.208, -0.190, UPPER_ROD_Z, y=y_val))
        rear_bracket = rear_bracket.cut(_x_cylinder(0.0090, -0.208, -0.190, LOWER_ROD_Z, y=y_val))
    for bolt_y in (-0.034, 0.034):
        for bolt_z in (AXIS_Z - 0.034, AXIS_Z + 0.034):
            rear_bracket = rear_bracket.cut(_x_cylinder(0.0052, -0.208, -0.190, bolt_z, y=bolt_y))

    rear_frame_shape = (
        base.union(rear_bracket)
        .union(_gusset([(-0.252, 0.014), (-0.206, 0.014), (-0.206, 0.116)], 0.046, 0.014))
        .union(_gusset([(-0.252, 0.014), (-0.206, 0.014), (-0.206, 0.116)], -0.060, 0.014))
    )

    front_yoke_shape = _box(0.016, 0.112, 0.154, x0=0.232, z0=0.023)
    front_yoke_shape = front_yoke_shape.union(_box(0.024, 0.090, 0.009, x0=0.224, z0=0.014))
    for y_val in (-rod_y, rod_y):
        front_yoke_shape = front_yoke_shape.cut(_x_cylinder(0.0090, 0.230, 0.248, UPPER_ROD_Z, y=y_val))
        front_yoke_shape = front_yoke_shape.cut(_x_cylinder(0.0090, 0.230, 0.248, LOWER_ROD_Z, y=y_val))
    front_yoke_shape = front_yoke_shape.cut(_box(0.020, 0.060, 0.060, x0=0.230, z0=AXIS_Z - 0.030))

    body_shell_shape = _x_tube(0.038, 0.028, -0.176, 0.164)
    flange_block = _box(0.016, 0.126, 0.126, x0=-0.192, z0=AXIS_Z - 0.063)
    flange_block = flange_block.cut(_x_cylinder(0.0285, -0.194, -0.174))
    for y_val in (-rod_y, rod_y):
        flange_block = flange_block.cut(_x_cylinder(0.0092, -0.194, -0.174, UPPER_ROD_Z, y=y_val))
        flange_block = flange_block.cut(_x_cylinder(0.0092, -0.194, -0.174, LOWER_ROD_Z, y=y_val))
    side_pad = _box(0.134, 0.008, 0.108, x0=-0.012, y=0.040, z0=0.046)
    slot_pos = _box(0.120, 0.028, 0.096, x0=-0.004, y=0.040, z0=0.052)
    slot_neg = _box(0.120, 0.028, 0.096, x0=-0.004, y=-0.040, z0=0.052)
    front_gland = _x_tube(0.046, 0.0226, 0.164, 0.178)
    front_hex = _x_hex(0.094, 0.178, 0.198, AXIS_Z).cut(_x_cylinder(0.0226, 0.176, 0.200))
    body_shell_shape = body_shell_shape.union(flange_block).union(side_pad).union(front_gland).union(front_hex)
    body_shell_shape = body_shell_shape.cut(slot_pos).cut(slot_neg)

    cover_shape = _box(0.134, 0.006, 0.108, x0=-0.012, y=0.047, z0=0.046)
    for bolt_x in (0.004, 0.056, 0.108):
        for bolt_z in (0.060, AXIS_Z, 0.140):
            cover_shape = cover_shape.union(_y_cylinder(0.0026, 0.043, 0.047, x=bolt_x, z=bolt_z))
            cover_shape = cover_shape.union(_y_cylinder(0.0046, 0.047, 0.053, x=bolt_x, z=bolt_z))

    def _guide_rod_shape(z_level: float, y_level: float) -> cq.Workplane:
        rod = _x_cylinder(0.0075, -0.190, 0.232, z_level, y=y_level)
        rod = rod.union(_x_cylinder(0.0102, -0.206, -0.190, z_level, y=y_level))
        rod = rod.union(_x_cylinder(0.0102, 0.248, 0.262, z_level, y=y_level))
        rod = rod.union(_x_hex(0.022, -0.220, -0.206, z_level, y=y_level))
        rod = rod.union(_x_hex(0.022, 0.262, 0.276, z_level, y=y_level))
        return rod

    upper_rod_shape = _guide_rod_shape(UPPER_ROD_Z, rod_y)
    lower_rod_shape = _guide_rod_shape(LOWER_ROD_Z, -rod_y)
    rear_frame_shape = rear_frame_shape.union(upper_rod_shape).union(lower_rod_shape)

    primary_body_shape = _x_tube(0.0215, 0.0162, -0.040, 0.182)
    primary_body_shape = primary_body_shape.union(_x_tube(0.0235, 0.0162, -0.084, -0.040))
    primary_body_shape = primary_body_shape.union(_x_tube(0.0180, 0.0110, 0.170, 0.182))
    carriage_shape = _box(0.096, 0.020, 0.024, x0=0.022, z0=UPPER_ROD_Z - 0.012)
    carriage_shape = carriage_shape.cut(_x_cylinder(0.0084, 0.018, 0.124, UPPER_ROD_Z, y=rod_y))
    lower_carriage = _box(0.096, 0.020, 0.024, x0=0.022, z0=LOWER_ROD_Z - 0.012)
    lower_carriage = lower_carriage.cut(_x_cylinder(0.0084, 0.018, 0.124, LOWER_ROD_Z, y=-rod_y))
    carriage_shape = carriage_shape.union(lower_carriage)
    carriage_shape = carriage_shape.union(_box(0.088, 0.012, UPPER_ROD_Z - LOWER_ROD_Z - 0.024, x0=0.026, y=0.022, z0=LOWER_ROD_Z + 0.012))
    carriage_shape = carriage_shape.union(_box(0.088, 0.012, UPPER_ROD_Z - LOWER_ROD_Z - 0.024, x0=0.026, y=-0.022, z0=LOWER_ROD_Z + 0.012))
    primary_body_shape = primary_body_shape.union(carriage_shape)
    primary_stop_shape = _x_tube(0.0300, 0.0162, 0.202, 0.214)
    primary_body_shape = primary_body_shape.union(primary_stop_shape)

    secondary_body_shape = _x_cylinder(0.0150, -0.050, 0.180)
    secondary_body_shape = secondary_body_shape.union(_x_cylinder(0.0105, 0.180, 0.320))
    secondary_body_shape = secondary_body_shape.union(_x_tube(0.0220, 0.0105, 0.206, 0.220))

    clevis_shape = _x_cylinder(0.0140, 0.320, 0.344)
    clevis_shape = clevis_shape.union(_x_hex(0.030, 0.306, 0.320, AXIS_Z).cut(_x_cylinder(0.0110, 0.304, 0.322)))
    clevis_shape = clevis_shape.union(_box(0.040, 0.040, 0.056, x0=0.332, z0=0.072))
    clevis_shape = clevis_shape.cut(_box(0.028, 0.018, 0.036, x0=0.344, z0=0.082))
    clevis_shape = clevis_shape.union(_y_cylinder(0.0050, -0.019, 0.019, x=0.356))

    support_frame = model.part("support_frame")
    support_frame.visual(_mesh(rear_frame_shape, "support_frame_rear.obj"), material=frame_mat, name="rear_frame")
    support_frame.visual(_mesh(front_yoke_shape, "support_frame_front_yoke.obj"), material=frame_mat, name="front_yoke")

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(_mesh(body_shell_shape, "outer_sleeve_body.obj"), material=steel_mat, name="body_shell")

    access_cover = model.part("access_cover")
    access_cover.visual(_mesh(cover_shape, "access_cover.obj"), material=frame_mat, name="cover_plate")

    primary_plunger = model.part("primary_plunger")
    primary_plunger.visual(_mesh(primary_body_shape, "primary_plunger_body.obj"), material=steel_mat, name="plunger_body")
    primary_plunger.visual(_mesh(carriage_shape, "primary_carriage.obj"), material=bronze_mat, name="guide_carriage")
    primary_plunger.visual(_mesh(primary_stop_shape, "primary_stop_collar.obj"), material=steel_mat, name="stop_collar")

    secondary_plunger = model.part("secondary_plunger")
    secondary_plunger.visual(_mesh(secondary_body_shape, "secondary_plunger.obj"), material=bright_steel, name="rod_body")

    terminal_clevis = model.part("terminal_clevis")
    terminal_clevis.visual(_mesh(clevis_shape, "terminal_clevis.obj"), material=steel_mat, name="clevis_assembly")

    zero = Origin(xyz=(0.0, 0.0, 0.0))
    model.articulation("frame_to_outer_sleeve", ArticulationType.FIXED, parent=support_frame, child=outer_sleeve, origin=zero)
    model.articulation("outer_sleeve_to_access_cover", ArticulationType.FIXED, parent=outer_sleeve, child=access_cover, origin=zero)
    model.articulation(
        "outer_sleeve_to_primary_plunger",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=primary_plunger,
        origin=zero,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.40, lower=0.0, upper=PRIMARY_TRAVEL),
    )
    model.articulation(
        "primary_plunger_to_secondary_plunger",
        ArticulationType.PRISMATIC,
        parent=primary_plunger,
        child=secondary_plunger,
        origin=zero,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.45, lower=0.0, upper=SECONDARY_TRAVEL),
    )
    model.articulation(
        "secondary_plunger_to_terminal_clevis",
        ArticulationType.FIXED,
        parent=secondary_plunger,
        child=terminal_clevis,
        origin=zero,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support_frame = object_model.get_part("support_frame")
    outer_sleeve = object_model.get_part("outer_sleeve")
    access_cover = object_model.get_part("access_cover")
    primary_plunger = object_model.get_part("primary_plunger")
    secondary_plunger = object_model.get_part("secondary_plunger")
    terminal_clevis = object_model.get_part("terminal_clevis")

    front_yoke = support_frame.get_visual("front_yoke")
    body_shell = outer_sleeve.get_visual("body_shell")
    cover_plate = access_cover.get_visual("cover_plate")
    primary_body = primary_plunger.get_visual("plunger_body")
    guide_carriage = primary_plunger.get_visual("guide_carriage")
    primary_stop = primary_plunger.get_visual("stop_collar")
    secondary_body = secondary_plunger.get_visual("rod_body")
    clevis_body = terminal_clevis.get_visual("clevis_assembly")

    primary_slide = object_model.get_articulation("outer_sleeve_to_primary_plunger")
    secondary_slide = object_model.get_articulation("primary_plunger_to_secondary_plunger")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(access_cover, outer_sleeve, reason="access cover is intentionally seated flush into the service opening")
    ctx.allow_overlap(outer_sleeve, support_frame, reason="outer sleeve rear flange is clamped into the frame bracket with seated interference")
    ctx.allow_overlap(outer_sleeve, primary_plunger, reason="primary plunger body runs as a nested telescoping fit through the sleeve gland and internal guide lands")
    ctx.allow_overlap(primary_plunger, secondary_plunger, reason="secondary rod telescopes concentrically inside the primary plunger sleeve")
    ctx.allow_overlap(secondary_plunger, terminal_clevis, reason="terminal clevis shank is rigidly inserted onto the secondary output rod as a fixed stud fit")
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(outer_sleeve, support_frame)
    ctx.expect_contact(access_cover, outer_sleeve, elem_a=cover_plate, elem_b=body_shell, contact_tol=0.001)
    ctx.expect_contact(terminal_clevis, secondary_plunger, elem_a=clevis_body, elem_b=secondary_body)

    ctx.expect_origin_distance(primary_plunger, outer_sleeve, axes="yz", max_dist=0.001)
    ctx.expect_origin_distance(secondary_plunger, primary_plunger, axes="yz", max_dist=0.001)
    ctx.expect_gap(primary_plunger, outer_sleeve, axis="x", positive_elem=primary_stop, negative_elem=body_shell, min_gap=0.003, max_gap=0.020)
    ctx.expect_within(secondary_plunger, primary_plunger, axes="yz", inner_elem=secondary_body, outer_elem=primary_body, margin=0.004)

    with ctx.pose({primary_slide: 0.060}):
        ctx.expect_gap(primary_plunger, outer_sleeve, axis="x", positive_elem=primary_stop, negative_elem=body_shell, min_gap=0.055)
        ctx.expect_gap(support_frame, primary_plunger, axis="x", positive_elem=front_yoke, negative_elem=guide_carriage, min_gap=0.040)

    with ctx.pose({primary_slide: PRIMARY_TRAVEL, secondary_slide: SECONDARY_TRAVEL}):
        ctx.expect_within(secondary_plunger, primary_plunger, axes="yz", inner_elem=secondary_body, outer_elem=primary_body, margin=0.004)
        ctx.expect_gap(support_frame, primary_plunger, axis="x", positive_elem=front_yoke, negative_elem=guide_carriage, min_gap=0.020)
        ctx.expect_gap(terminal_clevis, support_frame, axis="x", positive_elem=clevis_body, negative_elem=front_yoke, min_gap=0.180)

    primary_rest = ctx.part_world_position(primary_plunger)
    secondary_rest = ctx.part_world_position(secondary_plunger)
    clevis_rest = ctx.part_world_position(terminal_clevis)
    with ctx.pose({primary_slide: PRIMARY_TRAVEL, secondary_slide: SECONDARY_TRAVEL}):
        primary_full = ctx.part_world_position(primary_plunger)
        secondary_full = ctx.part_world_position(secondary_plunger)
        clevis_full = ctx.part_world_position(terminal_clevis)

    if primary_rest and secondary_rest and clevis_rest and primary_full and secondary_full and clevis_full:
        tol = 1e-6
        ctx.check(
            "primary_prismatic_motion_axis",
            abs((primary_full[0] - primary_rest[0]) - PRIMARY_TRAVEL) <= tol
            and abs(primary_full[1] - primary_rest[1]) <= tol
            and abs(primary_full[2] - primary_rest[2]) <= tol,
            details=f"rest={primary_rest}, full={primary_full}",
        )
        ctx.check(
            "secondary_prismatic_motion_axis",
            abs((secondary_full[0] - secondary_rest[0]) - (PRIMARY_TRAVEL + SECONDARY_TRAVEL)) <= tol
            and abs(secondary_full[1] - secondary_rest[1]) <= tol
            and abs(secondary_full[2] - secondary_rest[2]) <= tol,
            details=f"rest={secondary_rest}, full={secondary_full}",
        )
        ctx.check(
            "terminal_output_motion_axis",
            abs((clevis_full[0] - clevis_rest[0]) - (PRIMARY_TRAVEL + SECONDARY_TRAVEL)) <= tol
            and abs(clevis_full[1] - clevis_rest[1]) <= tol
            and abs(clevis_full[2] - clevis_rest[2]) <= tol,
            details=f"rest={clevis_rest}, full={clevis_full}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
