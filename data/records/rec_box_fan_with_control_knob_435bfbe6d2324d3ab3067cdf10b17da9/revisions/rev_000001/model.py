from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_ceiling_exhaust_fan")

    ceiling_white = model.material("ceiling_white", rgba=(0.95, 0.95, 0.94, 1.0))
    grille_white = model.material("grille_white", rgba=(0.92, 0.93, 0.94, 1.0))
    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.14, 0.15, 1.0))
    plastic_grey = model.material("plastic_grey", rgba=(0.68, 0.69, 0.71, 1.0))
    filter_white = model.material("filter_white", rgba=(0.97, 0.97, 0.95, 1.0))

    plate_outer = 0.340
    opening = 0.230
    plate_thickness = 0.012
    trim_band = (plate_outer - opening) * 0.5

    housing_outer = 0.228
    housing_wall = 0.006
    housing_wall_height = 0.164
    housing_top = 0.006
    flange_size = 0.260
    flange_thickness = 0.004

    ceiling_plate = model.part("ceiling_plate")
    ceiling_plate.visual(
        Box((plate_outer, trim_band, plate_thickness)),
        origin=Origin(xyz=(0.0, (plate_outer + opening) * 0.25, 0.0)),
        material=ceiling_white,
        name="front_trim",
    )
    ceiling_plate.visual(
        Box((plate_outer, trim_band, plate_thickness)),
        origin=Origin(xyz=(0.0, -(plate_outer + opening) * 0.25, 0.0)),
        material=ceiling_white,
        name="rear_trim",
    )
    ceiling_plate.visual(
        Box((trim_band, opening, plate_thickness)),
        origin=Origin(xyz=((plate_outer + opening) * 0.25, 0.0, 0.0)),
        material=ceiling_white,
        name="right_trim",
    )
    ceiling_plate.visual(
        Box((trim_band, opening, plate_thickness)),
        origin=Origin(xyz=(-(plate_outer + opening) * 0.25, 0.0, 0.0)),
        material=ceiling_white,
        name="left_trim",
    )
    for side, x_pos in (("left", -0.094), ("right", 0.094)):
        ceiling_plate.visual(
            Box((0.018, 0.014, 0.010)),
            origin=Origin(xyz=(x_pos, 0.136, 0.005)),
            material=galvanized,
            name=f"hinge_ear_{side}",
        )
    ceiling_plate.inertial = Inertial.from_geometry(
        Box((plate_outer, plate_outer, plate_thickness)),
        mass=1.3,
        origin=Origin(),
    )

    housing = model.part("housing")
    housing.visual(
        Box((flange_size, flange_size, flange_thickness)),
        origin=Origin(xyz=(0.0, 0.0, housing_top + (flange_thickness * 0.5))),
        material=galvanized,
        name="mount_flange",
    )
    housing.visual(
        Box((housing_outer, housing_outer, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, housing_top + housing_wall_height + 0.003)),
        material=galvanized,
        name="top_panel",
    )
    housing.visual(
        Box((housing_outer, housing_wall, housing_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                (housing_outer * 0.5) - (housing_wall * 0.5),
                housing_top + (housing_wall_height * 0.5),
            )
        ),
        material=galvanized,
        name="front_wall",
    )
    housing.visual(
        Box((housing_outer, housing_wall, housing_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -((housing_outer * 0.5) - (housing_wall * 0.5)),
                housing_top + (housing_wall_height * 0.5),
            )
        ),
        material=galvanized,
        name="rear_wall",
    )
    housing.visual(
        Box((housing_wall, housing_outer, housing_wall_height)),
        origin=Origin(
            xyz=(
                (housing_outer * 0.5) - (housing_wall * 0.5),
                0.0,
                housing_top + (housing_wall_height * 0.5),
            )
        ),
        material=galvanized,
        name="right_wall",
    )
    housing.visual(
        Box((housing_wall, housing_outer, housing_wall_height)),
        origin=Origin(
            xyz=(
                -((housing_outer * 0.5) - (housing_wall * 0.5)),
                0.0,
                housing_top + (housing_wall_height * 0.5),
            )
        ),
        material=galvanized,
        name="left_wall",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(-0.096, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_bearing",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.096, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_bearing",
    )
    housing.visual(
        Box((0.010, 0.070, 0.050)),
        origin=Origin(xyz=(0.119, 0.060, 0.114)),
        material=galvanized,
        name="control_pad",
    )
    housing.inertial = Inertial.from_geometry(
        Box((flange_size, flange_size, housing_top + housing_wall_height + 0.006)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
    )

    grille_panel = model.part("grille_panel")
    panel_size = 0.286
    panel_depth = panel_size
    frame_band = 0.014
    panel_thickness = 0.020
    support_span = 0.200
    grille_panel.visual(
        Box((panel_size, frame_band, panel_thickness)),
        origin=Origin(xyz=(0.0, -(frame_band * 0.5), -(panel_thickness * 0.5))),
        material=grille_white,
        name="front_frame",
    )
    grille_panel.visual(
        Box((panel_size, frame_band, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(panel_depth - (frame_band * 0.5)),
                -(panel_thickness * 0.5),
            )
        ),
        material=grille_white,
        name="rear_frame",
    )
    grille_panel.visual(
        Box((frame_band, panel_depth - (2.0 * frame_band), panel_thickness)),
        origin=Origin(
            xyz=(
                (panel_size * 0.5) - (frame_band * 0.5),
                -(panel_depth * 0.5),
                -(panel_thickness * 0.5),
            )
        ),
        material=grille_white,
        name="right_frame",
    )
    grille_panel.visual(
        Box((frame_band, panel_depth - (2.0 * frame_band), panel_thickness)),
        origin=Origin(
            xyz=(
                -((panel_size * 0.5) - (frame_band * 0.5)),
                -(panel_depth * 0.5),
                -(panel_thickness * 0.5),
            )
        ),
        material=grille_white,
        name="left_frame",
    )
    grille_panel.visual(
        Box((panel_size - (2.0 * frame_band), panel_depth - (2.0 * frame_band), 0.002)),
        origin=Origin(xyz=(0.0, -(panel_depth * 0.5), -0.015)),
        material=filter_white,
        name="filter_pad",
    )
    grille_panel.visual(
        Box((0.012, support_span, 0.006)),
        origin=Origin(xyz=(-0.094, -(panel_depth * 0.5), -0.012)),
        material=grille_white,
        name="left_louver_rail",
    )
    grille_panel.visual(
        Box((0.012, support_span, 0.006)),
        origin=Origin(xyz=(0.094, -(panel_depth * 0.5), -0.012)),
        material=grille_white,
        name="right_louver_rail",
    )
    for index, y_pos in enumerate((-0.050, -0.082, -0.114, -0.146, -0.178, -0.210)):
        grille_panel.visual(
            Box((0.202, 0.010, 0.005)),
            origin=Origin(xyz=(0.0, y_pos, -0.0125)),
            material=grille_white,
            name=f"slat_{index}",
        )
    for side, x_pos in (("left", -0.076), ("right", 0.076)):
        grille_panel.visual(
            Cylinder(radius=0.004, length=0.022),
            origin=Origin(
                xyz=(x_pos, -0.004, -0.008),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=galvanized,
            name=f"hinge_knuckle_{side}",
        )
    grille_panel.inertial = Inertial.from_geometry(
        Box((panel_size, panel_depth, panel_thickness)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -(panel_depth * 0.5), -(panel_thickness * 0.5))),
    )

    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        Cylinder(radius=0.005, length=0.168),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    blower_wheel.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    blower_wheel.visual(
        Cylinder(radius=0.055, length=0.004),
        origin=Origin(xyz=(-0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="left_end_disc",
    )
    blower_wheel.visual(
        Cylinder(radius=0.055, length=0.004),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="right_end_disc",
    )
    for index in range(14):
        angle = (math.tau * index) / 14.0
        blower_wheel.visual(
            Box((0.076, 0.018, 0.004)),
            origin=Origin(
                xyz=(0.0, 0.045 * math.cos(angle), 0.045 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=satin_black,
            name=f"blade_{index}",
        )
    blower_wheel.inertial = Inertial.from_geometry(
        Box((0.168, 0.114, 0.114)),
        mass=0.42,
        origin=Origin(),
    )

    humidity_dial = model.part("humidity_dial")
    humidity_dial.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_grey,
        name="dial_body",
    )
    humidity_dial.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_white,
        name="dial_cap",
    )
    humidity_dial.visual(
        Box((0.006, 0.004, 0.014)),
        origin=Origin(xyz=(0.021, 0.0, 0.013)),
        material=dark_steel,
        name="pointer",
    )
    humidity_dial.inertial = Inertial.from_geometry(
        Box((0.028, 0.040, 0.040)),
        mass=0.05,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
    )

    model.articulation(
        "ceiling_plate_to_housing",
        ArticulationType.FIXED,
        parent=ceiling_plate,
        child=housing,
        origin=Origin(),
    )
    model.articulation(
        "ceiling_plate_to_grille",
        ArticulationType.REVOLUTE,
        parent=ceiling_plate,
        child=grille_panel,
        origin=Origin(xyz=(0.0, panel_size * 0.5, -(plate_thickness * 0.5))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "housing_to_blower_wheel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=40.0,
            lower=-math.tau,
            upper=math.tau,
        ),
    )
    model.articulation(
        "housing_to_humidity_dial",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=humidity_dial,
        origin=Origin(xyz=(0.124, 0.060, 0.114)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=3.0,
            lower=-1.4,
            upper=1.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ceiling_plate = object_model.get_part("ceiling_plate")
    housing = object_model.get_part("housing")
    grille_panel = object_model.get_part("grille_panel")
    blower_wheel = object_model.get_part("blower_wheel")
    humidity_dial = object_model.get_part("humidity_dial")

    grille_hinge = object_model.get_articulation("ceiling_plate_to_grille")
    wheel_joint = object_model.get_articulation("housing_to_blower_wheel")
    dial_joint = object_model.get_articulation("housing_to_humidity_dial")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.expect_contact(
        housing,
        ceiling_plate,
        elem_a="mount_flange",
        name="housing flange seats on the ceiling plate",
    )
    ctx.expect_gap(
        ceiling_plate,
        grille_panel,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        name="closed grille sits flush under the ceiling plate",
    )
    ctx.expect_overlap(
        grille_panel,
        ceiling_plate,
        axes="xy",
        min_overlap=0.24,
        name="grille panel covers the ceiling opening footprint",
    )
    ctx.expect_contact(
        blower_wheel,
        housing,
        elem_a="axle",
        name="blower wheel axle seats in the housing bearings",
    )
    ctx.expect_within(
        blower_wheel,
        housing,
        axes="yz",
        margin=0.0,
        name="blower wheel stays inside the housing cavity",
    )
    ctx.expect_contact(
        humidity_dial,
        housing,
        elem_a="dial_body",
        elem_b="control_pad",
        name="humidity dial is mounted to the housing side pad",
    )

    ctx.check(
        "grille hinge uses a front-edge x-axis pivot",
        tuple(round(value, 4) for value in grille_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={grille_hinge.axis}",
    )
    ctx.check(
        "blower wheel axle is lateral",
        tuple(round(value, 4) for value in wheel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={wheel_joint.axis}",
    )
    dial_limits = dial_joint.motion_limits
    ctx.check(
        "humidity dial has limited rotary travel",
        dial_limits is not None
        and dial_limits.lower is not None
        and dial_limits.upper is not None
        and dial_limits.lower < 0.0
        and dial_limits.upper > 0.0
        and dial_limits.upper - dial_limits.lower <= 3.2,
        details=f"limits={dial_limits}",
    )

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    rear_closed = ctx.part_element_world_aabb(grille_panel, elem="rear_frame")
    with ctx.pose({grille_hinge: 1.25}):
        rear_open = ctx.part_element_world_aabb(grille_panel, elem="rear_frame")
    ctx.check(
        "grille rear edge swings downward for filter access",
        rear_closed is not None
        and rear_open is not None
        and _aabb_center(rear_open)[2] < _aabb_center(rear_closed)[2] - 0.12,
        details=f"closed={rear_closed}, open={rear_open}",
    )

    pointer_closed = ctx.part_element_world_aabb(humidity_dial, elem="pointer")
    with ctx.pose({dial_joint: 1.0}):
        pointer_open = ctx.part_element_world_aabb(humidity_dial, elem="pointer")
    ctx.check(
        "humidity dial pointer rotates around the side-face axle",
        pointer_closed is not None
        and pointer_open is not None
        and abs(_aabb_center(pointer_open)[1] - _aabb_center(pointer_closed)[1]) > 0.008,
        details=f"closed={pointer_closed}, open={pointer_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
