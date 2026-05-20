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
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

SHOULDER_AXIS_Z = 0.18
ELBOW_OFFSET_X = 0.22
ELBOW_OFFSET_Z = 0.03


def _ring_along_y(
    *,
    outer_r: float,
    inner_r: float,
    length: float,
    center_y: float,
    center_x: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(length)
        .translate((center_x, center_y + (length / 2.0), center_z))
    )


def _disc_along_y(
    *,
    radius: float,
    length: float,
    center_y: float,
    center_x: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x, center_y + (length / 2.0), center_z))
    )


def _disc_along_x(
    *,
    radius: float,
    length: float,
    center_x: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
    inner_r: float | None = None,
) -> cq.Workplane:
    wp = cq.Workplane("YZ").circle(radius)
    if inner_r is not None:
        wp = wp.circle(inner_r)
    return wp.extrude(length).translate((center_x - (length / 2.0), center_y, center_z))


def _cover_with_bolts(
    *,
    size_x: float,
    size_z: float,
    thickness: float,
    center_y: float,
    center_x: float,
    center_z: float,
    bolt_x: float,
    bolt_z: float,
    bolt_radius: float,
    bolt_height: float,
    outward_sign: float,
) -> cq.Workplane:
    cover = cq.Workplane("XY").box(size_x, thickness, size_z).translate((center_x, center_y, center_z))
    bolt_start_y = center_y + (thickness / 2.0) if outward_sign > 0.0 else center_y - (thickness / 2.0) - bolt_height
    for x_sign in (-1.0, 1.0):
        for z_sign in (-1.0, 1.0):
            bolt = (
                cq.Workplane("XZ")
                .circle(bolt_radius)
                .extrude(bolt_height)
                .translate((center_x + (x_sign * bolt_x), bolt_start_y, center_z + (z_sign * bolt_z)))
            )
            cover = cover.union(bolt)
    return cover


def _pedestal_main() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.18, 0.14, 0.018, centered=(True, True, False))
    for x_pos in (-0.055, 0.055):
        for y_pos in (-0.04, 0.04):
            slot = cq.Workplane("XY").slot2D(0.018, 0.008).extrude(0.03).translate((x_pos, y_pos, -0.001))
            base = base.cut(slot)

    column = cq.Workplane("XY").box(0.06, 0.10, 0.12).translate((0.0, 0.0, 0.078))
    bridge = cq.Workplane("XY").box(0.062, 0.11, 0.03).translate((0.0, 0.0, 0.147))
    left_cheek = cq.Workplane("XY").box(0.072, 0.012, 0.094).translate((0.0, 0.045, SHOULDER_AXIS_Z))
    right_cheek = cq.Workplane("XY").box(0.072, 0.012, 0.094).translate((0.0, -0.045, SHOULDER_AXIS_Z))

    front_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.018, 0.018),
                (0.018, 0.018),
                (0.032, 0.11),
                (0.032, 0.155),
                (-0.008, 0.155),
                (-0.026, 0.085),
            ]
        )
        .close()
        .extrude(0.012)
        .translate((0.0, 0.016, 0.0))
    )
    rear_gusset = front_gusset.translate((0.0, -0.044, 0.0))

    stop_left = cq.Workplane("XY").box(0.012, 0.010, 0.024).translate((-0.018, 0.028, 0.156))
    stop_right = cq.Workplane("XY").box(0.012, 0.010, 0.024).translate((-0.018, -0.028, 0.156))

    structure = (
        base.union(column)
        .union(bridge)
        .union(left_cheek)
        .union(right_cheek)
        .union(front_gusset)
        .union(rear_gusset)
        .union(stop_left)
        .union(stop_right)
    )

    shoulder_clearance = (
        cq.Workplane("XZ")
        .circle(0.026)
        .extrude(0.14)
        .translate((0.0, -0.07, SHOULDER_AXIS_Z))
    )
    return structure.cut(shoulder_clearance)


def _upper_arm_shoulder_hub() -> cq.Workplane:
    barrel = _disc_along_y(radius=0.0195, length=0.068, center_y=0.0, center_z=0.0)
    stop_lug = cq.Workplane("XY").box(0.020, 0.024, 0.018).translate((-0.014, 0.0, -0.024))
    return barrel.union(stop_lug)


def _upper_arm_main() -> cq.Workplane:
    prox_shift_x = 0.042
    left_plate = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, -0.022),
                (0.0, 0.022),
                (0.08, 0.028),
                (0.19, 0.038),
                (0.232, 0.036),
                (0.232, -0.010),
                (0.19, -0.018),
                (0.06, -0.026),
            ]
        )
        .close()
        .extrude(0.008)
        .translate((prox_shift_x, 0.020, 0.0))
    )
    right_plate = left_plate.translate((0.0, -0.048, 0.0))

    left_window = (
        cq.Workplane("XZ")
        .polyline([(0.05, -0.008), (0.05, 0.010), (0.12, 0.015), (0.14, -0.012)])
        .close()
        .extrude(0.010)
        .translate((prox_shift_x, 0.019, 0.0))
    )
    right_window = left_window.translate((0.0, -0.048, 0.0))
    left_plate = left_plate.cut(left_window)
    right_plate = right_plate.cut(right_window)

    shoulder_block = cq.Workplane("XY").box(0.050, 0.036, 0.032).translate((0.025 + prox_shift_x, 0.0, 0.0))
    top_web = cq.Workplane("XY").box(0.135, 0.038, 0.012).translate((0.108 + prox_shift_x, 0.0, 0.018))
    bottom_web = cq.Workplane("XY").box(0.110, 0.030, 0.010).translate((0.100 + prox_shift_x, 0.0, -0.016))

    left_cheek = cq.Workplane("XY").box(0.044, 0.010, 0.056).translate((ELBOW_OFFSET_X, 0.035, ELBOW_OFFSET_Z))
    right_cheek = cq.Workplane("XY").box(0.044, 0.010, 0.056).translate((ELBOW_OFFSET_X, -0.035, ELBOW_OFFSET_Z))
    bridge_top = cq.Workplane("XY").box(0.044, 0.050, 0.010).translate((ELBOW_OFFSET_X, 0.0, ELBOW_OFFSET_Z + 0.023))
    bridge_bottom = cq.Workplane("XY").box(0.034, 0.038, 0.010).translate((ELBOW_OFFSET_X, 0.0, ELBOW_OFFSET_Z - 0.022))

    body = (
        left_plate.union(right_plate)
        .union(shoulder_block)
        .union(top_web)
        .union(bottom_web)
        .union(left_cheek)
        .union(right_cheek)
        .union(bridge_top)
        .union(bridge_bottom)
    )

    elbow_clearance = (
        cq.Workplane("XZ")
        .circle(0.024)
        .extrude(0.12)
        .translate((ELBOW_OFFSET_X, -0.06, ELBOW_OFFSET_Z))
    )
    return body.cut(elbow_clearance)


def _forearm_elbow_hub() -> cq.Workplane:
    barrel = _disc_along_y(radius=0.0195, length=0.056, center_y=0.0, center_z=0.0)
    stop_lug = cq.Workplane("XY").box(0.018, 0.020, 0.015).translate((-0.012, 0.0, -0.020))
    return barrel.union(stop_lug)


def _forearm_main() -> cq.Workplane:
    prox_shift_x = 0.030
    left_plate = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, -0.016),
                (0.0, 0.020),
                (0.05, 0.024),
                (0.14, 0.020),
                (0.175, 0.014),
                (0.175, -0.010),
                (0.135, -0.018),
                (0.055, -0.022),
            ]
        )
        .close()
        .extrude(0.007)
        .translate((prox_shift_x, 0.015, 0.0))
    )
    right_plate = left_plate.translate((0.0, -0.037, 0.0))

    left_window = (
        cq.Workplane("XZ")
        .polyline([(0.040, -0.004), (0.040, 0.010), (0.090, 0.012), (0.112, -0.008)])
        .close()
        .extrude(0.009)
        .translate((prox_shift_x, 0.014, 0.0))
    )
    right_window = left_window.translate((0.0, -0.037, 0.0))
    left_plate = left_plate.cut(left_window)
    right_plate = right_plate.cut(right_window)

    proximal_box = cq.Workplane("XY").box(0.030, 0.030, 0.028).translate((0.015 + prox_shift_x, 0.0, 0.0))
    top_web = cq.Workplane("XY").box(0.110, 0.028, 0.010).translate((0.085 + prox_shift_x, 0.0, 0.015))
    bottom_web = cq.Workplane("XY").box(0.100, 0.024, 0.008).translate((0.082 + prox_shift_x, 0.0, -0.016))
    flange_neck = cq.Workplane("XY").box(0.022, 0.026, 0.024).translate((0.172 + prox_shift_x, 0.0, 0.0))

    return left_plate.union(right_plate).union(proximal_box).union(top_web).union(bottom_web).union(flange_neck)


def _end_flange() -> cq.Workplane:
    flange = _disc_along_x(radius=0.028, inner_r=0.010, length=0.012, center_x=0.190, center_y=0.0, center_z=0.0)
    for y_pos, z_pos in ((0.018, 0.0), (-0.018, 0.0), (0.0, 0.018), (0.0, -0.018)):
        hole = cq.Workplane("YZ").circle(0.0032).extrude(0.02).translate((0.180, y_pos, z_pos))
        flange = flange.cut(hole)
    return flange


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serial_elbow_arm", assets=ASSETS)

    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.35, 0.37, 0.40, 1.0))
    bolt_dark = model.material("bolt_dark", rgba=(0.19, 0.20, 0.22, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_main(), "pedestal_main.obj", assets=ASSETS),
        material=steel,
        name="main_structure",
    )
    pedestal.visual(
        mesh_from_cadquery(
            _ring_along_y(outer_r=0.032, inner_r=0.0205, length=0.008, center_y=0.035, center_z=SHOULDER_AXIS_Z),
            "pedestal_left_bearing.obj",
            assets=ASSETS,
        ),
        material=bolt_dark,
        name="left_bearing",
    )
    pedestal.visual(
        mesh_from_cadquery(
            _ring_along_y(outer_r=0.032, inner_r=0.0205, length=0.008, center_y=-0.035, center_z=SHOULDER_AXIS_Z),
            "pedestal_right_bearing.obj",
            assets=ASSETS,
        ),
        material=bolt_dark,
        name="right_bearing",
    )
    pedestal.visual(
        mesh_from_cadquery(
            _cover_with_bolts(
                size_x=0.058,
                size_z=0.072,
                thickness=0.004,
                center_y=0.053,
                center_x=0.0,
                center_z=SHOULDER_AXIS_Z,
                bolt_x=0.018,
                bolt_z=0.022,
                bolt_radius=0.0032,
                bolt_height=0.003,
                outward_sign=1.0,
            ),
            "pedestal_left_cover.obj",
            assets=ASSETS,
        ),
        material=cover_gray,
        name="left_cover",
    )
    pedestal.visual(
        mesh_from_cadquery(
            _cover_with_bolts(
                size_x=0.058,
                size_z=0.072,
                thickness=0.004,
                center_y=-0.053,
                center_x=0.0,
                center_z=SHOULDER_AXIS_Z,
                bolt_x=0.018,
                bolt_z=0.022,
                bolt_radius=0.0032,
                bolt_height=0.003,
                outward_sign=-1.0,
            ),
            "pedestal_right_cover.obj",
            assets=ASSETS,
        ),
        material=cover_gray,
        name="right_cover",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 0.24)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shoulder_hub(), "upper_arm_shoulder_hub.obj", assets=ASSETS),
        material=bolt_dark,
        name="shoulder_hub",
    )
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_main(), "upper_arm_main.obj", assets=ASSETS),
        material=steel,
        name="main_structure",
    )
    upper_arm.visual(
        mesh_from_cadquery(
            _ring_along_y(
                outer_r=0.029,
                inner_r=0.0205,
                length=0.006,
                center_y=0.027,
                center_x=ELBOW_OFFSET_X,
                center_z=ELBOW_OFFSET_Z,
            ),
            "upper_arm_left_elbow_bearing.obj",
            assets=ASSETS,
        ),
        material=bolt_dark,
        name="left_elbow_bearing",
    )
    upper_arm.visual(
        mesh_from_cadquery(
            _ring_along_y(
                outer_r=0.029,
                inner_r=0.0205,
                length=0.006,
                center_y=-0.027,
                center_x=ELBOW_OFFSET_X,
                center_z=ELBOW_OFFSET_Z,
            ),
            "upper_arm_right_elbow_bearing.obj",
            assets=ASSETS,
        ),
        material=bolt_dark,
        name="right_elbow_bearing",
    )
    upper_arm.visual(
        mesh_from_cadquery(
            _cover_with_bolts(
                size_x=0.036,
                size_z=0.050,
                thickness=0.0035,
                center_y=0.0415,
                center_x=ELBOW_OFFSET_X,
                center_z=ELBOW_OFFSET_Z,
                bolt_x=0.011,
                bolt_z=0.015,
                bolt_radius=0.0028,
                bolt_height=0.0025,
                outward_sign=1.0,
            ),
            "upper_arm_left_elbow_cover.obj",
            assets=ASSETS,
        ),
        material=cover_gray,
        name="left_elbow_cover",
    )
    upper_arm.visual(
        mesh_from_cadquery(
            _cover_with_bolts(
                size_x=0.036,
                size_z=0.050,
                thickness=0.0035,
                center_y=-0.0415,
                center_x=ELBOW_OFFSET_X,
                center_z=ELBOW_OFFSET_Z,
                bolt_x=0.011,
                bolt_z=0.015,
                bolt_radius=0.0028,
                bolt_height=0.0025,
                outward_sign=-1.0,
            ),
            "upper_arm_right_elbow_cover.obj",
            assets=ASSETS,
        ),
        material=cover_gray,
        name="right_elbow_cover",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.24, 0.08, 0.08)),
        mass=4.8,
        origin=Origin(xyz=(0.12, 0.0, 0.01)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_elbow_hub(), "forearm_elbow_hub.obj", assets=ASSETS),
        material=bolt_dark,
        name="elbow_hub",
    )
    forearm.visual(
        mesh_from_cadquery(_forearm_main(), "forearm_main.obj", assets=ASSETS),
        material=steel,
        name="main_structure",
    )
    forearm.visual(
        mesh_from_cadquery(_end_flange(), "forearm_end_flange.obj", assets=ASSETS),
        material=cover_gray,
        name="end_flange",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.21, 0.06, 0.06)),
        mass=2.9,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-1.2, upper=0.9),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(ELBOW_OFFSET_X, 0.0, ELBOW_OFFSET_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-2.1, upper=0.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")

    pedestal_main = pedestal.get_visual("main_structure")
    left_bearing = pedestal.get_visual("left_bearing")
    right_bearing = pedestal.get_visual("right_bearing")
    shoulder_hub = upper_arm.get_visual("shoulder_hub")
    upper_main = upper_arm.get_visual("main_structure")
    left_elbow_bearing = upper_arm.get_visual("left_elbow_bearing")
    right_elbow_bearing = upper_arm.get_visual("right_elbow_bearing")
    elbow_hub = forearm.get_visual("elbow_hub")
    forearm_main = forearm.get_visual("main_structure")
    end_flange = forearm.get_visual("end_flange")

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
    # `expect_gap(...)`, `expect_overlap(...)`, or `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "part_presence",
        pedestal is not None and upper_arm is not None and forearm is not None,
        "Pedestal, upper arm, and forearm parts must all exist.",
    )
    ctx.check(
        "shoulder_joint_axis",
        tuple(round(v, 6) for v in shoulder.axis) == (0.0, 1.0, 0.0),
        f"Unexpected shoulder axis: {shoulder.axis}",
    )
    ctx.check(
        "elbow_joint_axis",
        tuple(round(v, 6) for v in elbow.axis) == (0.0, 1.0, 0.0),
        f"Unexpected elbow axis: {elbow.axis}",
    )

    ctx.expect_within(
        upper_arm,
        pedestal,
        axes="z",
        inner_elem=shoulder_hub,
        outer_elem=left_bearing,
        margin=0.0,
        name="shoulder_hub_within_left_bearing_height",
    )
    ctx.expect_within(
        upper_arm,
        pedestal,
        axes="z",
        inner_elem=shoulder_hub,
        outer_elem=right_bearing,
        margin=0.0,
        name="shoulder_hub_within_right_bearing_height",
    )
    ctx.expect_overlap(
        upper_arm,
        pedestal,
        axes="y",
        elem_a=shoulder_hub,
        elem_b=left_bearing,
        min_overlap=0.002,
        name="left_shoulder_bearing_engagement",
    )
    ctx.expect_overlap(
        upper_arm,
        pedestal,
        axes="y",
        elem_a=shoulder_hub,
        elem_b=right_bearing,
        min_overlap=0.002,
        name="right_shoulder_bearing_engagement",
    )

    ctx.expect_within(
        forearm,
        upper_arm,
        axes="z",
        inner_elem=elbow_hub,
        outer_elem=left_elbow_bearing,
        margin=0.0,
        name="elbow_hub_within_left_bearing_height",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="z",
        inner_elem=elbow_hub,
        outer_elem=right_elbow_bearing,
        margin=0.0,
        name="elbow_hub_within_right_bearing_height",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="y",
        elem_a=elbow_hub,
        elem_b=left_elbow_bearing,
        min_overlap=0.001,
        name="left_elbow_bearing_engagement",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="y",
        elem_a=elbow_hub,
        elem_b=right_elbow_bearing,
        min_overlap=0.001,
        name="right_elbow_bearing_engagement",
    )

    pedestal_aabb = ctx.part_world_aabb(pedestal)
    upper_aabb = ctx.part_world_aabb(upper_arm)
    forearm_aabb = ctx.part_world_aabb(forearm)
    flange_aabb = ctx.part_element_world_aabb(forearm, elem=end_flange)
    ctx.check(
        "assembly_has_forward_reach",
        pedestal_aabb is not None and forearm_aabb is not None and (forearm_aabb[1][0] - pedestal_aabb[0][0]) > 0.36,
        f"Unexpected reach extents pedestal={pedestal_aabb} forearm={forearm_aabb}",
    )
    ctx.check(
        "upper_arm_extends_beyond_pedestal",
        upper_aabb is not None and pedestal_aabb is not None and upper_aabb[1][0] > pedestal_aabb[1][0] + 0.08,
        f"Upper arm should project well beyond pedestal. pedestal={pedestal_aabb} upper={upper_aabb}",
    )
    ctx.check(
        "flange_is_compact",
        flange_aabb is not None and (flange_aabb[1][0] - flange_aabb[0][0]) < 0.02,
        f"End flange should remain compact. flange={flange_aabb}",
    )

    rest_flange_top_z = flange_aabb[1][2] if flange_aabb is not None else None
    with ctx.pose({shoulder: -0.7, elbow: -1.2}):
        folded_flange_aabb = ctx.part_element_world_aabb(forearm, elem=end_flange)
        folded_forearm_aabb = ctx.part_world_aabb(forearm)
        ctx.check(
            "folded_pose_lifts_end_flange",
            rest_flange_top_z is not None
            and folded_flange_aabb is not None
            and folded_flange_aabb[1][2] > rest_flange_top_z + 0.10,
            f"Folded pose should lift flange. rest={rest_flange_top_z} folded={folded_flange_aabb}",
        )
        ctx.check(
            "folded_pose_keeps_forearm_clear_of_base",
            folded_forearm_aabb is not None and pedestal_aabb is not None and folded_forearm_aabb[0][2] > pedestal_aabb[1][2] - 0.01,
            f"Forearm should stay above the base deck when folded. folded={folded_forearm_aabb} pedestal={pedestal_aabb}",
        )

    with ctx.pose({shoulder: 0.45, elbow: -0.35}):
        lowered_flange_aabb = ctx.part_element_world_aabb(forearm, elem=end_flange)
        ctx.check(
            "deployment_pose_pushes_flange_forward",
            lowered_flange_aabb is not None and lowered_flange_aabb[1][0] > 0.32,
            f"Deployment pose should keep the flange projecting forward. flange={lowered_flange_aabb}",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=16)
    ctx.expect_overlap(
        upper_arm,
        pedestal,
        axes="xz",
        elem_a=shoulder_hub,
        elem_b=pedestal_main,
        min_overlap=0.01,
        name="shoulder_axis_reads_as_integrated_in_pedestal",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="xz",
        elem_a=elbow_hub,
        elem_b=upper_main,
        min_overlap=0.008,
        name="elbow_axis_reads_as_integrated_in_upper_arm",
    )
    ctx.expect_contact(
        forearm,
        forearm,
        elem_a=forearm_main,
        elem_b=end_flange,
        contact_tol=1e-6,
        name="end_flange_is_built_onto_forearm",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
