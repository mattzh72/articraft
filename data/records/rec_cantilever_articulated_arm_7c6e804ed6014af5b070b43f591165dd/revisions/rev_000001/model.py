from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.240
BASE_WIDTH = 0.180
BASE_HEIGHT = 0.020

TOWER_WIDTH = 0.090
TOWER_DEPTH = 0.064
TOWER_COLUMN_HEIGHT = 0.176
TOP_PAD_RADIUS = 0.045
TOP_PAD_HEIGHT = 0.014
TOWER_TOP_Z = BASE_HEIGHT + TOWER_COLUMN_HEIGHT + TOP_PAD_HEIGHT

UPPER_LINK_LENGTH = 0.405
UPPER_LINK_Z = 0.030
FORELINK_LENGTH = 0.265
WRIST_BODY_LENGTH = 0.145
FACEPLATE_THICKNESS = 0.008

UPPER_CLEVIS_GAP = 0.054
FORE_CLEVIS_GAP = 0.046
CHEEK_THICKNESS = 0.010


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _z_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - length / 2.0))
    )


def _tower_shape() -> cq.Workplane:
    base = _box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, (0.0, 0.0, BASE_HEIGHT / 2.0))
    base = base.edges("|Z").fillet(0.010)

    column = _box(
        TOWER_WIDTH,
        TOWER_DEPTH,
        TOWER_COLUMN_HEIGHT,
        (0.0, 0.0, BASE_HEIGHT + TOWER_COLUMN_HEIGHT / 2.0),
    )
    column = column.edges("|Z").fillet(0.010)

    top_pad = _z_cylinder(
        TOP_PAD_RADIUS,
        TOP_PAD_HEIGHT,
        (0.0, 0.0, BASE_HEIGHT + TOWER_COLUMN_HEIGHT + TOP_PAD_HEIGHT / 2.0),
    )
    front_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.030, BASE_HEIGHT),
                (0.030, BASE_HEIGHT),
                (0.030, BASE_HEIGHT + 0.050),
                (0.012, BASE_HEIGHT + 0.120),
                (-0.012, BASE_HEIGHT + 0.120),
                (-0.030, BASE_HEIGHT + 0.050),
            ]
        )
        .close()
        .extrude(0.020)
        .translate((0.0, -0.010, 0.0))
    )
    rear_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.030, BASE_HEIGHT),
                (0.030, BASE_HEIGHT),
                (0.030, BASE_HEIGHT + 0.050),
                (0.012, BASE_HEIGHT + 0.118),
                (-0.012, BASE_HEIGHT + 0.118),
                (-0.030, BASE_HEIGHT + 0.050),
            ]
        )
        .close()
        .extrude(0.020)
        .translate((0.0, BASE_WIDTH / 2.0 - 0.010, 0.0))
    )

    tower = base.union(column).union(top_pad).union(front_gusset).union(rear_gusset)

    hole_points = [
        (-0.075, -0.050),
        (-0.075, 0.050),
        (0.075, -0.050),
        (0.075, 0.050),
    ]
    tower = (
        tower.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(hole_points)
        .hole(0.012)
    )
    return tower


def _upper_link_shape() -> cq.Workplane:
    collar = _z_cylinder(0.030, 0.050, (0.0, 0.0, 0.025))
    collar_flange = _z_cylinder(0.038, 0.010, (0.0, 0.0, 0.005))

    beam = _box(0.340, 0.046, 0.036, (0.195, 0.0, UPPER_LINK_Z))
    beam = beam.edges("|X").fillet(0.005)
    top_rib = _box(0.200, 0.024, 0.012, (0.205, 0.0, UPPER_LINK_Z + 0.020))

    root_shoulder = _box(0.080, 0.056, 0.042, (0.040, 0.0, UPPER_LINK_Z))

    cheek_y = UPPER_CLEVIS_GAP / 2.0 + CHEEK_THICKNESS / 2.0
    cheek_left = _box(0.030, CHEEK_THICKNESS, 0.056, (UPPER_LINK_LENGTH - 0.015, cheek_y, 0.030))
    cheek_right = _box(0.030, CHEEK_THICKNESS, 0.056, (UPPER_LINK_LENGTH - 0.015, -cheek_y, 0.030))
    clevis_bridge = _box(0.046, 0.074, 0.010, (UPPER_LINK_LENGTH - 0.023, 0.0, 0.055))
    clevis_riser = _box(0.034, 0.032, 0.022, (UPPER_LINK_LENGTH - 0.058, 0.0, 0.043))

    return (
        collar.union(collar_flange)
        .union(root_shoulder)
        .union(beam)
        .union(top_rib)
        .union(cheek_left)
        .union(cheek_right)
        .union(clevis_bridge)
        .union(clevis_riser)
    )


def _forelink_shape() -> cq.Workplane:
    elbow_barrel = _y_cylinder(0.019, UPPER_CLEVIS_GAP, (0.019, 0.0, 0.0))
    root_block = _box(0.058, 0.044, 0.036, (0.029, 0.0, 0.0))

    beam = _box(0.190, 0.036, 0.032, (0.145, 0.0, 0.0))
    beam = beam.edges("|X").fillet(0.004)
    top_rib = _box(0.112, 0.018, 0.010, (0.148, 0.0, 0.017))

    cheek_y = FORE_CLEVIS_GAP / 2.0 + CHEEK_THICKNESS / 2.0
    cheek_left = _box(0.022, CHEEK_THICKNESS, 0.054, (FORELINK_LENGTH - 0.011, cheek_y, 0.0))
    cheek_right = _box(0.022, CHEEK_THICKNESS, 0.054, (FORELINK_LENGTH - 0.011, -cheek_y, 0.0))
    clevis_bridge = _box(0.036, 0.064, 0.010, (FORELINK_LENGTH - 0.021, 0.0, 0.028))
    clevis_riser = _box(0.026, 0.026, 0.020, (FORELINK_LENGTH - 0.040, 0.0, 0.018))

    return (
        elbow_barrel.union(root_block)
        .union(beam)
        .union(top_rib)
        .union(cheek_left)
        .union(cheek_right)
        .union(clevis_bridge)
        .union(clevis_riser)
    )


def _wrist_body_shape() -> cq.Workplane:
    wrist_barrel = _y_cylinder(0.016, FORE_CLEVIS_GAP, (0.016, 0.0, 0.0))
    root_block = _box(0.044, 0.030, 0.026, (0.022, 0.0, 0.0))
    body = _box(0.118, 0.026, 0.030, (0.079, 0.0, 0.0))
    body = body.edges("|X").fillet(0.003)
    top_rib = _box(0.064, 0.014, 0.008, (0.090, 0.0, 0.016))
    nose = _box(0.018, 0.022, 0.026, (0.136, 0.0, 0.0))
    return wrist_barrel.union(root_block).union(body).union(top_rib).union(nose)


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) / 2.0 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mounted_tooling_arm")

    model.material("tower_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("arm_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("forelink_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("wrist_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("plate_steel", rgba=(0.84, 0.85, 0.87, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_tower_shape(), "tower_body"),
        material="tower_charcoal",
        name="tower_body",
    )
    tower.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, TOWER_TOP_Z)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z / 2.0)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_upper_link_shape(), "upper_link_body"),
        material="arm_aluminum",
        name="upper_link_body",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((UPPER_LINK_LENGTH, 0.080, 0.070)),
        mass=6.2,
        origin=Origin(xyz=(UPPER_LINK_LENGTH * 0.46, 0.0, 0.030)),
    )

    forelink = model.part("forelink")
    forelink.visual(
        mesh_from_cadquery(_forelink_shape(), "forelink_body"),
        material="forelink_aluminum",
        name="forelink_body",
    )
    forelink.inertial = Inertial.from_geometry(
        Box((FORELINK_LENGTH, 0.062, 0.060)),
        mass=3.4,
        origin=Origin(xyz=(FORELINK_LENGTH * 0.45, 0.0, 0.0)),
    )

    wrist = model.part("wrist_cartridge")
    wrist.visual(
        mesh_from_cadquery(_wrist_body_shape(), "wrist_body"),
        material="wrist_dark",
        name="wrist_body",
    )
    wrist.visual(
        Box((FACEPLATE_THICKNESS, 0.062, 0.088)),
        origin=Origin(xyz=(WRIST_BODY_LENGTH + FACEPLATE_THICKNESS / 2.0 - 0.0005, 0.0, 0.0)),
        material="plate_steel",
        name="faceplate",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((WRIST_BODY_LENGTH + FACEPLATE_THICKNESS, 0.090, 0.090)),
        mass=1.7,
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
    )

    model.articulation(
        "tower_yaw",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=1.2,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "upper_to_forelink",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, UPPER_LINK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.55,
            upper=1.15,
        ),
    )
    model.articulation(
        "forelink_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=wrist,
        origin=Origin(xyz=(FORELINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-1.35,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    tower = object_model.get_part("tower")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    wrist = object_model.get_part("wrist_cartridge")

    tower_yaw = object_model.get_articulation("tower_yaw")
    upper_to_forelink = object_model.get_articulation("upper_to_forelink")
    forelink_to_wrist = object_model.get_articulation("forelink_to_wrist")

    ctx.check(
        "joint stack uses intended axes",
        tower_yaw.axis == (0.0, 0.0, 1.0)
        and upper_to_forelink.axis == (0.0, -1.0, 0.0)
        and forelink_to_wrist.axis == (0.0, -1.0, 0.0),
        details=(
            f"tower_yaw={tower_yaw.axis}, "
            f"upper_to_forelink={upper_to_forelink.axis}, "
            f"forelink_to_wrist={forelink_to_wrist.axis}"
        ),
    )

    ctx.expect_gap(
        wrist,
        tower,
        axis="x",
        positive_elem="faceplate",
        negative_elem="tower_body",
        min_gap=0.55,
        name="faceplate projects well forward of the tower",
    )

    rest_wrist_pos = ctx.part_world_position(wrist)
    with ctx.pose({tower_yaw: 0.80}):
        yawed_wrist_pos = ctx.part_world_position(wrist)
    ctx.check(
        "tower yaw swings the arm to the side",
        rest_wrist_pos is not None
        and yawed_wrist_pos is not None
        and yawed_wrist_pos[1] > rest_wrist_pos[1] + 0.35,
        details=f"rest={rest_wrist_pos}, yawed={yawed_wrist_pos}",
    )

    with ctx.pose({tower_yaw: 0.25, upper_to_forelink: 0.0}):
        rest_elbow_wrist_pos = ctx.part_world_position(wrist)
    with ctx.pose({tower_yaw: 0.25, upper_to_forelink: 0.90}):
        lifted_elbow_wrist_pos = ctx.part_world_position(wrist)
    ctx.check(
        "elbow pitch raises the wrist cartridge",
        rest_elbow_wrist_pos is not None
        and lifted_elbow_wrist_pos is not None
        and lifted_elbow_wrist_pos[2] > rest_elbow_wrist_pos[2] + 0.14,
        details=f"rest={rest_elbow_wrist_pos}, raised={lifted_elbow_wrist_pos}",
    )

    with ctx.pose({forelink_to_wrist: 0.0}):
        neutral_plate = _aabb_center(ctx.part_element_world_aabb(wrist, elem="faceplate"))
    with ctx.pose({forelink_to_wrist: 0.90}):
        raised_plate = _aabb_center(ctx.part_element_world_aabb(wrist, elem="faceplate"))
    ctx.check(
        "wrist pitch lifts the faceplate edge",
        neutral_plate is not None
        and raised_plate is not None
        and raised_plate[2] > neutral_plate[2] + 0.08,
        details=f"neutral={neutral_plate}, raised={raised_plate}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
