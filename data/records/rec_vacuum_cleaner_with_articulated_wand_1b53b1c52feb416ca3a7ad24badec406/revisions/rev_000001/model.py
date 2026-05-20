from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _straight_tube(start, end, *, radius: float, segments: int = 16) -> MeshGeometry:
    return tube_from_spline_points(
        [start, end],
        radius=radius,
        samples_per_segment=2,
        radial_segments=segments,
        cap_ends=True,
    )


def _wand_loop(length: float, *, half_width: float, z_offset: float, radius: float) -> MeshGeometry:
    return wire_from_points(
        [
            (0.065, -half_width, z_offset),
            (length - 0.065, -half_width, z_offset),
            (length - 0.065, half_width, z_offset),
            (0.065, half_width, z_offset),
        ],
        radius=radius,
        radial_segments=16,
        closed_path=True,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.030,
        corner_segments=8,
    )


def _add_child_lug(part, *, material, barrel_name: str, web_name: str, y_length: float = 0.080) -> None:
    part.visual(
        Cylinder(radius=0.038, length=y_length),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=barrel_name,
    )
    part.visual(
        Box((0.082, 0.060, 0.034)),
        origin=Origin(xyz=(0.038, 0.0, 0.022)),
        material=material,
        name=web_name,
    )


def _add_parent_yoke(
    part,
    *,
    x: float,
    material,
    cheek_names: tuple[str, str],
    web_names: tuple[str, str],
    cap_names: tuple[str, str],
    cheek_inner: float = 0.040,
) -> None:
    cheek_y = cheek_inner + 0.012
    for index, side in enumerate((-1.0, 1.0)):
        y = side * cheek_y
        part.visual(
            Box((0.060, 0.024, 0.092)),
            origin=Origin(xyz=(x, y, 0.0)),
            material=material,
            name=cheek_names[index],
        )
        part.visual(
            Box((0.080, 0.024, 0.036)),
            origin=Origin(xyz=(x - 0.040, y, 0.030)),
            material=material,
            name=web_names[index],
        )
        part.visual(
            Cylinder(radius=0.043, length=0.010),
            origin=Origin(xyz=(x, side * (cheek_inner + 0.029), 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=material,
            name=cap_names[index],
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_articulated_vacuum")

    graphite = model.material("graphite_plastic", rgba=(0.09, 0.10, 0.11, 1.0))
    charcoal = model.material("charcoal_rubber", rgba=(0.025, 0.026, 0.028, 1.0))
    blue_shell = model.material("blue_motor_shell", rgba=(0.10, 0.22, 0.42, 1.0))
    smoky_cup = model.material("smoky_clear_cup", rgba=(0.50, 0.70, 0.82, 0.42))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.77, 1.0))
    hinge_dark = model.material("dark_hinge_metal", rgba=(0.20, 0.21, 0.22, 1.0))
    warning_red = model.material("red_release_tabs", rgba=(0.70, 0.05, 0.04, 1.0))
    brush = model.material("brush_strip", rgba=(0.98, 0.75, 0.22, 1.0))

    main_body = model.part("main_body")
    main_body.inertial = Inertial.from_geometry(
        Box((0.58, 0.28, 0.26)),
        mass=3.2,
        origin=Origin(xyz=(-0.31, 0.0, 0.075)),
    )
    main_body.visual(
        _save_mesh(CapsuleGeometry(radius=0.115, length=0.265), "motor_pod"),
        origin=Origin(xyz=(-0.355, 0.0, 0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue_shell,
        name="motor_pod",
    )
    main_body.visual(
        Cylinder(radius=0.092, length=0.190),
        origin=Origin(xyz=(-0.180, 0.0, -0.105), rpy=(0.0, pi / 2.0, 0.0)),
        material=smoky_cup,
        name="dust_cup",
    )
    main_body.visual(
        Cylinder(radius=0.078, length=0.010),
        origin=Origin(xyz=(-0.078, 0.0, -0.105), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="cup_lip",
    )
    main_body.visual(
        Cylinder(radius=0.085, length=0.020),
        origin=Origin(xyz=(-0.565, 0.0, 0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_filter_grille",
    )
    for y in (-0.020, 0.020):
        main_body.visual(
            Box((0.012, 0.115, 0.010)),
            origin=Origin(xyz=(-0.580, y, 0.115)),
            material=aluminum,
            name=f"grille_slat_{0 if y < 0 else 1}",
        )
    handle_mesh = tube_from_spline_points(
        [
            (-0.520, 0.0, 0.205),
            (-0.455, 0.0, 0.305),
            (-0.280, 0.0, 0.330),
            (-0.165, 0.0, 0.245),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    main_body.visual(_save_mesh(handle_mesh, "body_handle"), material=graphite, name="body_handle")
    main_body.visual(
        Box((0.100, 0.120, 0.045)),
        origin=Origin(xyz=(-0.100, 0.0, 0.018)),
        material=graphite,
        name="socket_backbone",
    )
    main_body.visual(
        Box((0.180, 0.050, 0.045)),
        origin=Origin(xyz=(-0.150, 0.0, -0.018)),
        material=graphite,
        name="cup_cradle",
    )
    for side, label in ((-1.0, "a"), (1.0, "b")):
        main_body.visual(
            Box((0.095, 0.022, 0.044)),
            origin=Origin(xyz=(-0.048, side * 0.052, 0.020)),
            material=graphite,
            name=f"socket_side_arm_{label}",
        )
    _add_parent_yoke(
        main_body,
        x=0.0,
        material=hinge_dark,
        cheek_names=("body_elbow_cheek_a", "body_elbow_cheek_b"),
        web_names=("body_elbow_side_web_a", "body_elbow_side_web_b"),
        cap_names=("body_elbow_pin_cap_a", "body_elbow_pin_cap_b"),
    )
    main_body.visual(
        Box((0.040, 0.055, 0.018)),
        origin=Origin(xyz=(-0.130, 0.0, 0.030)),
        material=warning_red,
        name="release_tab",
    )

    upper_wand = model.part("upper_wand")
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.55, 0.14, 0.08)),
        mass=0.55,
        origin=Origin(xyz=(0.275, 0.0, 0.020)),
    )
    _add_child_lug(
        upper_wand,
        material=hinge_dark,
        barrel_name="upper_proximal_barrel",
        web_name="upper_proximal_lug_web",
    )
    upper_frame = _merge_meshes(
        _wand_loop(0.55, half_width=0.047, z_offset=0.052, radius=0.0135),
        _straight_tube((0.115, -0.047, 0.052), (0.435, 0.047, 0.052), radius=0.0085, segments=12),
        _straight_tube((0.115, 0.047, 0.052), (0.435, -0.047, 0.052), radius=0.0085, segments=12),
    )
    upper_wand.visual(_save_mesh(upper_frame, "upper_open_frame"), material=aluminum, name="upper_open_frame")
    upper_wand.visual(
        Box((0.060, 0.064, 0.032)),
        origin=Origin(xyz=(0.070, 0.0, 0.038)),
        material=hinge_dark,
        name="upper_proximal_bridge",
    )
    _add_parent_yoke(
        upper_wand,
        x=0.55,
        material=hinge_dark,
        cheek_names=("wand_elbow_cheek_a", "wand_elbow_cheek_b"),
        web_names=("wand_elbow_side_web_a", "wand_elbow_side_web_b"),
        cap_names=("wand_elbow_pin_cap_a", "wand_elbow_pin_cap_b"),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.60, 0.14, 0.08)),
        mass=0.62,
        origin=Origin(xyz=(0.300, 0.0, 0.020)),
    )
    _add_child_lug(
        lower_wand,
        material=hinge_dark,
        barrel_name="lower_proximal_barrel",
        web_name="lower_proximal_lug_web",
    )
    lower_frame = _merge_meshes(
        _wand_loop(0.60, half_width=0.043, z_offset=0.050, radius=0.0125),
        _straight_tube((0.125, -0.043, 0.050), (0.475, 0.043, 0.050), radius=0.008, segments=12),
        _straight_tube((0.125, 0.043, 0.050), (0.475, -0.043, 0.050), radius=0.008, segments=12),
    )
    lower_wand.visual(_save_mesh(lower_frame, "lower_open_frame"), material=aluminum, name="lower_open_frame")
    lower_wand.visual(
        Box((0.064, 0.060, 0.030)),
        origin=Origin(xyz=(0.070, 0.0, 0.036)),
        material=hinge_dark,
        name="lower_proximal_bridge",
    )
    _add_parent_yoke(
        lower_wand,
        x=0.60,
        material=hinge_dark,
        cheek_names=("nozzle_hinge_cheek_a", "nozzle_hinge_cheek_b"),
        web_names=("nozzle_hinge_side_web_a", "nozzle_hinge_side_web_b"),
        cap_names=("nozzle_hinge_pin_cap_a", "nozzle_hinge_pin_cap_b"),
        cheek_inner=0.045,
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.25, 0.36, 0.08)),
        mass=0.95,
        origin=Origin(xyz=(0.120, 0.0, -0.080)),
    )
    _add_child_lug(
        floor_nozzle,
        material=hinge_dark,
        barrel_name="nozzle_lug_barrel",
        web_name="nozzle_lug_web",
        y_length=0.090,
    )
    floor_nozzle.visual(
        Box((0.120, 0.070, 0.052)),
        origin=Origin(xyz=(0.058, 0.0, -0.045)),
        material=hinge_dark,
        name="nozzle_pitch_arm",
    )
    floor_nozzle.visual(
        _save_mesh(
            ExtrudeGeometry(rounded_rect_profile(0.245, 0.350, 0.030, corner_segments=8), 0.050),
            "nozzle_shell",
        ),
        origin=Origin(xyz=(0.130, 0.0, -0.087)),
        material=graphite,
        name="nozzle_shell",
    )
    floor_nozzle.visual(
        Box((0.170, 0.285, 0.010)),
        origin=Origin(xyz=(0.165, 0.0, -0.118)),
        material=charcoal,
        name="suction_mouth",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.018, length=0.275),
        origin=Origin(xyz=(0.165, 0.0, -0.107), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brush,
        name="brush_roll",
    )
    floor_nozzle.visual(
        Box((0.055, 0.335, 0.025)),
        origin=Origin(xyz=(0.030, 0.0, -0.075)),
        material=charcoal,
        name="rear_bumper",
    )
    for side, label in ((-1.0, "a"), (1.0, "b")):
        floor_nozzle.visual(
            Cylinder(radius=0.023, length=0.018),
            origin=Origin(xyz=(0.035, side * 0.178, -0.101), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=f"side_wheel_{label}",
        )

    model.articulation(
        "body_elbow",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=upper_wand,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.65, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-0.55, upper=0.85),
    )
    model.articulation(
        "wand_elbow",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.55, 0.0, 0.0), rpy=(0.0, -0.18, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.2, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "nozzle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.60, 0.0, 0.0), rpy=(0.0, -0.47, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.5, velocity=2.5, lower=-0.80, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    main_body = object_model.get_part("main_body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    body_elbow = object_model.get_articulation("body_elbow")
    wand_elbow = object_model.get_articulation("wand_elbow")
    nozzle_pitch = object_model.get_articulation("nozzle_pitch")

    ctx.check(
        "wand has two bending elbows and a nozzle pitch hinge",
        all(joint is not None for joint in (body_elbow, wand_elbow, nozzle_pitch)),
        details="Expected body_elbow, wand_elbow, and nozzle_pitch revolute joints.",
    )
    ctx.expect_contact(
        upper_wand,
        main_body,
        elem_a="upper_proximal_barrel",
        elem_b="body_elbow_cheek_a",
        name="upper wand lug is captured in the body yoke",
    )
    ctx.expect_contact(
        lower_wand,
        upper_wand,
        elem_a="lower_proximal_barrel",
        elem_b="wand_elbow_cheek_a",
        name="lower wand lug is captured in the exposed elbow yoke",
    )
    ctx.expect_contact(
        floor_nozzle,
        lower_wand,
        elem_a="nozzle_lug_barrel",
        elem_b="nozzle_hinge_cheek_a",
        name="floor nozzle lug is captured in the lower yoke",
    )

    rest_nozzle_pos = ctx.part_world_position(floor_nozzle)
    with ctx.pose({wand_elbow: 0.55}):
        bent_nozzle_pos = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "middle elbow moves the downstream wand and nozzle",
        rest_nozzle_pos is not None
        and bent_nozzle_pos is not None
        and abs(bent_nozzle_pos[2] - rest_nozzle_pos[2]) > 0.08,
        details=f"rest={rest_nozzle_pos}, bent={bent_nozzle_pos}",
    )

    rest_nozzle_aabb = ctx.part_world_aabb(floor_nozzle)
    with ctx.pose({nozzle_pitch: 0.55}):
        pitched_nozzle_aabb = ctx.part_world_aabb(floor_nozzle)
    ctx.check(
        "nozzle pitch hinge rotates the floor head about its horizontal pin",
        rest_nozzle_aabb is not None
        and pitched_nozzle_aabb is not None
        and abs(pitched_nozzle_aabb[0][2] - rest_nozzle_aabb[0][2]) > 0.050,
        details=f"rest={rest_nozzle_aabb}, pitched={pitched_nozzle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
