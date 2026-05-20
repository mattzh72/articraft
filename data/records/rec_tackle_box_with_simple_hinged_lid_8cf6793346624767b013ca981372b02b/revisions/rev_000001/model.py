from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_DEPTH = 0.280
BODY_WIDTH = 0.420
BODY_HEIGHT = 0.170
WALL_THICKNESS = 0.006
LID_TOP_THICKNESS = 0.004
LID_SKIRT_DEPTH = 0.028
HINGE_RADIUS = 0.006
HINGE_AXIS_X = -BODY_DEPTH * 0.5 - 0.002
HINGE_AXIS_Z = BODY_HEIGHT + HINGE_RADIUS


def _merge_meshes(*geometries):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _hinge_barrel(length: float, *, center_y: float, center_x: float = 0.0, center_z: float = 0.0):
    return (
        CylinderGeometry(radius=HINGE_RADIUS, height=length, radial_segments=28)
        .rotate_x(math.pi * 0.5)
        .translate(center_x, center_y, center_z)
    )


def _build_body_shell_mesh():
    floor = BoxGeometry((BODY_DEPTH, BODY_WIDTH, WALL_THICKNESS)).translate(
        0.0,
        0.0,
        WALL_THICKNESS * 0.5,
    )
    front_wall = BoxGeometry((WALL_THICKNESS, BODY_WIDTH, BODY_HEIGHT)).translate(
        BODY_DEPTH * 0.5 - WALL_THICKNESS * 0.5,
        0.0,
        BODY_HEIGHT * 0.5,
    )
    rear_wall = BoxGeometry((WALL_THICKNESS, BODY_WIDTH, BODY_HEIGHT)).translate(
        -BODY_DEPTH * 0.5 + WALL_THICKNESS * 0.5,
        0.0,
        BODY_HEIGHT * 0.5,
    )
    left_wall = BoxGeometry((BODY_DEPTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, BODY_HEIGHT)).translate(
        0.0,
        -BODY_WIDTH * 0.5 + WALL_THICKNESS * 0.5,
        BODY_HEIGHT * 0.5,
    )
    right_wall = BoxGeometry((BODY_DEPTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, BODY_HEIGHT)).translate(
        0.0,
        BODY_WIDTH * 0.5 - WALL_THICKNESS * 0.5,
        BODY_HEIGHT * 0.5,
    )
    front_rim = BoxGeometry((0.012, BODY_WIDTH, 0.010)).translate(
        BODY_DEPTH * 0.5 - 0.006,
        0.0,
        BODY_HEIGHT - 0.005,
    )
    left_rim = BoxGeometry((BODY_DEPTH - 0.020, 0.010, 0.010)).translate(
        0.0,
        -BODY_WIDTH * 0.5 + 0.005,
        BODY_HEIGHT - 0.005,
    )
    right_rim = BoxGeometry((BODY_DEPTH - 0.020, 0.010, 0.010)).translate(
        0.0,
        BODY_WIDTH * 0.5 - 0.005,
        BODY_HEIGHT - 0.005,
    )
    return _merge_meshes(floor, front_wall, rear_wall, left_wall, right_wall, front_rim, left_rim, right_rim)


def _build_reinforcement_band_mesh():
    band_height = 0.030
    band_top = BODY_HEIGHT - 0.014
    band_back = BoxGeometry((0.018, BODY_WIDTH + 0.020, band_height)).translate(
        -BODY_DEPTH * 0.5 - 0.009,
        0.0,
        band_top - band_height * 0.5,
    )
    return_height = 0.026
    left_return = BoxGeometry((0.020, 0.012, return_height)).translate(
        -BODY_DEPTH * 0.5 + 0.009,
        -BODY_WIDTH * 0.5 - 0.006,
        band_top - return_height * 0.5,
    )
    right_return = BoxGeometry((0.020, 0.012, return_height)).translate(
        -BODY_DEPTH * 0.5 + 0.009,
        BODY_WIDTH * 0.5 + 0.006,
        band_top - return_height * 0.5,
    )
    left_barrel = _hinge_barrel(0.072, center_y=-0.078, center_x=-0.002, center_z=0.000)
    right_barrel = _hinge_barrel(0.072, center_y=0.078, center_x=-0.002, center_z=0.000)
    left_hinge_block = BoxGeometry((0.014, 0.072, 0.016)).translate(-0.006, -0.078, -0.006)
    right_hinge_block = BoxGeometry((0.014, 0.072, 0.016)).translate(-0.006, 0.078, -0.006)
    hinge_mount = BoxGeometry((0.016, BODY_WIDTH + 0.010, 0.014)).translate(
        -0.012,
        0.0,
        -0.015,
    )
    hinge_cluster = _merge_meshes(left_barrel, right_barrel, left_hinge_block, right_hinge_block, hinge_mount)
    hinge_cluster.translate(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)
    return _merge_meshes(band_back, left_return, right_return, hinge_cluster)


def _build_tray_insert_mesh():
    tray_depth = BODY_DEPTH - 0.036
    tray_width = BODY_WIDTH - 0.036
    tray_height = 0.034
    base = BoxGeometry((tray_depth, tray_width, 0.004)).translate(0.0, 0.0, BODY_HEIGHT * 0.0 + 0.008)
    front_wall = BoxGeometry((0.004, tray_width, tray_height)).translate(
        tray_depth * 0.5 - 0.002,
        0.0,
        0.006 + tray_height * 0.5,
    )
    rear_wall = BoxGeometry((0.004, tray_width, tray_height)).translate(
        -tray_depth * 0.5 + 0.002,
        0.0,
        0.006 + tray_height * 0.5,
    )
    left_wall = BoxGeometry((tray_depth - 0.008, 0.004, tray_height)).translate(
        0.0,
        -tray_width * 0.5 + 0.002,
        0.006 + tray_height * 0.5,
    )
    right_wall = BoxGeometry((tray_depth - 0.008, 0.004, tray_height)).translate(
        0.0,
        tray_width * 0.5 - 0.002,
        0.006 + tray_height * 0.5,
    )
    center_divider = BoxGeometry((0.004, tray_width - 0.012, tray_height - 0.006)).translate(
        0.0,
        0.0,
        0.009 + (tray_height - 0.006) * 0.5,
    )
    side_divider = BoxGeometry((0.074, 0.004, tray_height - 0.010)).translate(
        0.035,
        0.080,
        0.011 + (tray_height - 0.010) * 0.5,
    )
    return _merge_meshes(base, front_wall, rear_wall, left_wall, right_wall, center_divider, side_divider)


def _build_lid_mesh():
    top_skin = BoxGeometry((BODY_DEPTH + 0.008, BODY_WIDTH + 0.012, LID_TOP_THICKNESS)).translate(
        BODY_DEPTH * 0.5,
        0.0,
        -0.004,
    )
    front_skirt = BoxGeometry((0.010, BODY_WIDTH + 0.012, LID_SKIRT_DEPTH)).translate(
        BODY_DEPTH + 0.004,
        0.0,
        -0.5 * LID_SKIRT_DEPTH,
    )
    left_skirt = BoxGeometry((BODY_DEPTH - 0.004, 0.010, LID_SKIRT_DEPTH)).translate(
        BODY_DEPTH * 0.5 + 0.002,
        -BODY_WIDTH * 0.5 - 0.006,
        -0.5 * LID_SKIRT_DEPTH,
    )
    right_skirt = BoxGeometry((BODY_DEPTH - 0.004, 0.010, LID_SKIRT_DEPTH)).translate(
        BODY_DEPTH * 0.5 + 0.002,
        BODY_WIDTH * 0.5 + 0.006,
        -0.5 * LID_SKIRT_DEPTH,
    )
    rear_backbone = BoxGeometry((0.012, BODY_WIDTH + 0.012, 0.018)).translate(
        -0.002,
        0.0,
        -0.009,
    )
    lid_knuckle_left = _hinge_barrel(0.072, center_y=-0.156)
    lid_knuckle_center = _hinge_barrel(0.072, center_y=0.0)
    lid_knuckle_right = _hinge_barrel(0.072, center_y=0.156)
    return _merge_meshes(
        top_skin,
        front_skirt,
        left_skirt,
        right_skirt,
        rear_backbone,
        lid_knuckle_left,
        lid_knuckle_center,
        lid_knuckle_right,
    )


def _author_reinforcement_band(part, material):
    band_back = part.visual(
        Box((0.018, BODY_WIDTH + 0.020, 0.032)),
        origin=Origin(
            xyz=(
                -BODY_DEPTH * 0.5 - 0.009,
                0.0,
                BODY_HEIGHT - 0.030,
            )
        ),
        material=material,
        name="band_back",
    )
    part.visual(
        Box((0.020, 0.012, 0.026)),
        origin=Origin(
            xyz=(
                -BODY_DEPTH * 0.5 + 0.009,
                -BODY_WIDTH * 0.5 - 0.006,
                BODY_HEIGHT - 0.027,
            )
        ),
        material=material,
        name="left_band_return",
    )
    part.visual(
        Box((0.020, 0.012, 0.026)),
        origin=Origin(
            xyz=(
                -BODY_DEPTH * 0.5 + 0.009,
                BODY_WIDTH * 0.5 + 0.006,
                BODY_HEIGHT - 0.027,
            )
        ),
        material=material,
        name="right_band_return",
    )
    part.visual(
        Box((0.014, 0.072, 0.032)),
        origin=Origin(xyz=(HINGE_AXIS_X - 0.011, -0.078, BODY_HEIGHT - 0.010)),
        material=material,
        name="left_hinge_ear",
    )
    part.visual(
        Box((0.014, 0.072, 0.032)),
        origin=Origin(xyz=(HINGE_AXIS_X - 0.011, 0.078, BODY_HEIGHT - 0.010)),
        material=material,
        name="right_hinge_ear",
    )
    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.072),
        origin=Origin(
            xyz=(HINGE_AXIS_X, -0.078, HINGE_AXIS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=material,
        name="left_band_barrel",
    )
    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.072),
        origin=Origin(
            xyz=(HINGE_AXIS_X, 0.078, HINGE_AXIS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=material,
        name="right_band_barrel",
    )
    return band_back


def _author_lid(part, material):
    lid_panel_depth = BODY_DEPTH
    panel_x_start = 0.008
    top_center_x = panel_x_start + lid_panel_depth * 0.5
    side_skirt_x_start = 0.030
    side_skirt_x_end = panel_x_start + lid_panel_depth - 0.010
    side_skirt_length = side_skirt_x_end - side_skirt_x_start
    side_skirt_center_x = side_skirt_x_start + side_skirt_length * 0.5

    part.visual(
        Box((lid_panel_depth, BODY_WIDTH + 0.012, LID_TOP_THICKNESS)),
        origin=Origin(xyz=(top_center_x, 0.0, -0.004)),
        material=material,
        name="lid_top",
    )
    part.visual(
        Box((0.010, BODY_WIDTH + 0.012, LID_SKIRT_DEPTH)),
        origin=Origin(
            xyz=(panel_x_start + lid_panel_depth - 0.005, 0.0, -0.5 * LID_SKIRT_DEPTH)
        ),
        material=material,
        name="front_skirt",
    )
    part.visual(
        Box((side_skirt_length, 0.010, LID_SKIRT_DEPTH)),
        origin=Origin(
            xyz=(side_skirt_center_x, -BODY_WIDTH * 0.5 - 0.006, -0.5 * LID_SKIRT_DEPTH)
        ),
        material=material,
        name="left_skirt",
    )
    part.visual(
        Box((side_skirt_length, 0.010, LID_SKIRT_DEPTH)),
        origin=Origin(
            xyz=(side_skirt_center_x, BODY_WIDTH * 0.5 + 0.006, -0.5 * LID_SKIRT_DEPTH)
        ),
        material=material,
        name="right_skirt",
    )

    for leaf_name, barrel_name, center_y in (
        ("left_hinge_leaf", "left_lid_knuckle", -0.156),
        ("center_hinge_leaf", "center_lid_knuckle", 0.0),
        ("right_hinge_leaf", "right_lid_knuckle", 0.156),
    ):
        part.visual(
            Box((0.020, 0.072, 0.012)),
            origin=Origin(xyz=(0.014, center_y, -0.006)),
            material=material,
            name=leaf_name,
        )
        part.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.072),
            origin=Origin(
                xyz=(0.0, center_y, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=material,
            name=barrel_name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tackle_box")

    body_material = model.material("body_material", rgba=(0.70, 0.74, 0.78, 1.0))
    band_material = model.material("band_material", rgba=(0.44, 0.47, 0.52, 1.0))
    tray_material = model.material("tray_material", rgba=(0.84, 0.85, 0.80, 1.0))
    lid_material = model.material("lid_material", rgba=(0.74, 0.78, 0.82, 1.0))

    body_shell = model.part("body_shell")
    body_shell.visual(
        mesh_from_geometry(_build_body_shell_mesh(), "body_shell_mesh"),
        material=body_material,
        name="shell_body",
    )
    body_shell.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    reinforcement_band = model.part("reinforcement_band")
    _author_reinforcement_band(reinforcement_band, band_material)
    reinforcement_band.inertial = Inertial.from_geometry(
        Box((0.036, BODY_WIDTH + 0.020, 0.040)),
        mass=0.45,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, BODY_HEIGHT - 0.030)),
    )

    tray_insert = model.part("tray_insert")
    tray_insert.visual(
        mesh_from_geometry(_build_tray_insert_mesh(), "tray_insert_mesh"),
        material=tray_material,
        name="tray_organizer",
    )
    tray_insert.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH - 0.036, BODY_WIDTH - 0.036, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    lid = model.part("lid")
    _author_lid(lid, lid_material)
    lid.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH, BODY_WIDTH + 0.012, 0.040)),
        mass=0.95,
        origin=Origin(xyz=(0.148, 0.0, -0.010)),
    )

    model.articulation(
        "shell_to_band",
        ArticulationType.FIXED,
        parent=body_shell,
        child=reinforcement_band,
        origin=Origin(),
    )
    model.articulation(
        "shell_to_tray",
        ArticulationType.FIXED,
        parent=body_shell,
        child=tray_insert,
        origin=Origin(),
    )
    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=body_shell,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body_shell = object_model.get_part("body_shell")
    reinforcement_band = object_model.get_part("reinforcement_band")
    tray_insert = object_model.get_part("tray_insert")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("shell_to_lid")

    ctx.expect_contact(
        reinforcement_band,
        body_shell,
        name="reinforcement band is mounted to the shell",
    )
    ctx.expect_contact(
        tray_insert,
        body_shell,
        name="tray insert is seated on the body shell",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body_shell,
            axis="z",
            positive_elem="lid_top",
            negative_elem="shell_body",
            max_gap=0.006,
            max_penetration=0.0,
            name="closed lid top sits on the box opening plane",
        )
        ctx.expect_overlap(
            lid,
            body_shell,
            axes="xy",
            min_overlap=0.200,
            name="closed lid covers the body footprint",
        )

    rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")
    with ctx.pose({lid_hinge: math.radians(100.0)}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")
        ctx.check(
            "lid opens upward",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > rest_aabb[1][2] + 0.10,
            details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
