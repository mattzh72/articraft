from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _annular_cylinder_mesh(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 96,
) -> MeshGeometry:
    """Connected hollow cylinder mesh used for clean-room rings and drum glass."""
    mesh = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer_bottom.append(mesh.add_vertex(outer_radius * ca, outer_radius * sa, z_min))
        outer_top.append(mesh.add_vertex(outer_radius * ca, outer_radius * sa, z_max))
        inner_bottom.append(mesh.add_vertex(inner_radius * ca, inner_radius * sa, z_min))
        inner_top.append(mesh.add_vertex(inner_radius * ca, inner_radius * sa, z_max))

    for index in range(segments):
        nxt = (index + 1) % segments
        _add_quad(mesh, outer_bottom[index], outer_bottom[nxt], outer_top[nxt], outer_top[index])
        _add_quad(mesh, inner_bottom[nxt], inner_bottom[index], inner_top[index], inner_top[nxt])
        _add_quad(mesh, outer_top[index], outer_top[nxt], inner_top[nxt], inner_top[index])
        _add_quad(mesh, outer_bottom[nxt], outer_bottom[index], inner_bottom[index], inner_bottom[nxt])
    return mesh


def _radial_origin(radius: float, angle: float, z: float) -> Origin:
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
        rpy=(0.0, 0.0, angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clean_room_three_wing_revolving_door")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.73, 1.0))
    darker_steel = model.material("dark_stainless", rgba=(0.36, 0.38, 0.39, 1.0))
    glass = model.material("clear_cleanroom_glass", rgba=(0.62, 0.82, 0.92, 0.30))
    laminated_panel = model.material("sealed_laminated_panel", rgba=(0.74, 0.88, 0.95, 0.55))
    gasket = model.material("black_epdm_seal", rgba=(0.02, 0.025, 0.025, 1.0))
    white_polymer = model.material("white_cleanroom_polymer", rgba=(0.92, 0.94, 0.93, 1.0))

    housing = model.part("drum_housing")
    housing.visual(
        Cylinder(radius=1.18, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stainless,
        name="floor_plinth",
    )
    housing.visual(
        mesh_from_geometry(
            _annular_cylinder_mesh(
                inner_radius=1.055,
                outer_radius=1.105,
                z_min=0.115,
                z_max=2.505,
                segments=128,
            ),
            "transparent_sealed_drum_shell",
        ),
        material=glass,
        name="sealed_drum_shell",
    )
    housing.visual(
        Cylinder(radius=1.18, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.585)),
        material=stainless,
        name="top_canopy",
    )
    housing.visual(
        mesh_from_geometry(
            _annular_cylinder_mesh(
                inner_radius=0.075,
                outer_radius=0.155,
                z_min=0.118,
                z_max=0.205,
                segments=64,
            ),
            "lower_post_bearing_ring",
        ),
        material=darker_steel,
        name="lower_bearing",
    )
    housing.visual(
        mesh_from_geometry(
            _annular_cylinder_mesh(
                inner_radius=0.075,
                outer_radius=0.155,
                z_min=2.415,
                z_max=2.505,
                segments=64,
            ),
            "upper_post_bearing_ring",
        ),
        material=darker_steel,
        name="upper_bearing",
    )
    for name, y_pos in (("front_seal_jamb", 1.112), ("rear_seal_jamb", -1.112)):
        housing.visual(
            Box((0.18, 0.055, 2.30)),
            origin=Origin(xyz=(0.0, y_pos, 1.31)),
            material=gasket,
            name=name,
        )
    for z_pos, visual_name in ((0.215, "lower_cleanroom_gasket"), (2.405, "upper_cleanroom_gasket")):
        housing.visual(
            mesh_from_geometry(
                _annular_cylinder_mesh(
                    inner_radius=1.025,
                    outer_radius=1.070,
                    z_min=z_pos - 0.018,
                    z_max=z_pos + 0.018,
                    segments=128,
                ),
                visual_name,
            ),
            material=gasket,
            name=visual_name,
        )
    housing.visual(
        Box((0.62, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, 1.18, 0.1475)),
        material=white_polymer,
        name="front_flush_threshold",
    )
    housing.visual(
        Box((0.62, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, -1.18, 0.1475)),
        material=white_polymer,
        name="rear_flush_threshold",
    )
    housing.inertial = Inertial.from_geometry(
        Cylinder(radius=1.18, length=2.66),
        mass=280.0,
        origin=Origin(xyz=(0.0, 0.0, 1.33)),
    )

    rotor = model.part("wing_assembly")
    rotor.visual(
        Cylinder(radius=0.055, length=2.36),
        origin=Origin(xyz=(0.0, 0.0, 1.31)),
        material=stainless,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.118, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=darker_steel,
        name="lower_hub",
    )
    rotor.visual(
        Cylinder(radius=0.118, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.365)),
        material=darker_steel,
        name="upper_hub",
    )

    wing_length = 0.86
    wing_root = 0.105
    wing_center = wing_root + wing_length * 0.5
    wing_outer = wing_root + wing_length
    sweep_root = 0.17
    sweep_length = wing_outer - sweep_root
    sweep_center = sweep_root + sweep_length * 0.5

    wing_0 = model.part("wing_0")
    angle_0 = 0.0
    wing_0.visual(
        Box((wing_length, 0.032, 1.82)),
        origin=_radial_origin(wing_center, angle_0, 1.31),
        material=laminated_panel,
        name="wing_panel_0",
    )
    wing_0.visual(
        Box((wing_length + 0.035, 0.060, 0.055)),
        origin=_radial_origin(wing_center, angle_0, 2.235),
        material=stainless,
        name="top_rail_0",
    )
    wing_0.visual(
        Box((wing_length + 0.035, 0.060, 0.055)),
        origin=_radial_origin(wing_center, angle_0, 0.385),
        material=stainless,
        name="bottom_rail_0",
    )
    wing_0.visual(
        Box((0.095, 0.070, 1.92)),
        origin=_radial_origin(0.1025, angle_0, 1.31),
        material=stainless,
        name="root_stile_0",
    )
    wing_0.visual(
        Box((0.045, 0.070, 1.94)),
        origin=_radial_origin(wing_outer, angle_0, 1.31),
        material=gasket,
        name="outer_seal_0",
    )
    wing_0.visual(
        Box((sweep_length, 0.012, 0.080)),
        origin=_radial_origin(sweep_center, angle_0, 0.325),
        material=gasket,
        name="sweep_seal_0",
    )
    wing_0.inertial = Inertial.from_geometry(
        Box((wing_outer, 0.08, 2.0)),
        mass=24.0,
        origin=Origin(xyz=(wing_center, 0.0, 1.31)),
    )

    wing_1 = model.part("wing_1")
    angle_1 = 2.0 * math.pi / 3.0
    wing_1.visual(
        Box((wing_length, 0.032, 1.82)),
        origin=_radial_origin(wing_center, angle_1, 1.31),
        material=laminated_panel,
        name="wing_panel_1",
    )
    wing_1.visual(
        Box((wing_length + 0.035, 0.060, 0.055)),
        origin=_radial_origin(wing_center, angle_1, 2.235),
        material=stainless,
        name="top_rail_1",
    )
    wing_1.visual(
        Box((wing_length + 0.035, 0.060, 0.055)),
        origin=_radial_origin(wing_center, angle_1, 0.385),
        material=stainless,
        name="bottom_rail_1",
    )
    wing_1.visual(
        Box((0.095, 0.070, 1.92)),
        origin=_radial_origin(0.1025, angle_1, 1.31),
        material=stainless,
        name="root_stile_1",
    )
    wing_1.visual(
        Box((0.045, 0.070, 1.94)),
        origin=_radial_origin(wing_outer, angle_1, 1.31),
        material=gasket,
        name="outer_seal_1",
    )
    wing_1.visual(
        Box((sweep_length, 0.012, 0.080)),
        origin=_radial_origin(sweep_center, angle_1, 0.325),
        material=gasket,
        name="sweep_seal_1",
    )
    wing_1.inertial = Inertial.from_geometry(
        Box((wing_outer, 0.08, 2.0)),
        mass=24.0,
        origin=Origin(xyz=(wing_center * math.cos(angle_1), wing_center * math.sin(angle_1), 1.31)),
    )

    wing_2 = model.part("wing_2")
    angle_2 = 4.0 * math.pi / 3.0
    wing_2.visual(
        Box((wing_length, 0.032, 1.82)),
        origin=_radial_origin(wing_center, angle_2, 1.31),
        material=laminated_panel,
        name="wing_panel_2",
    )
    wing_2.visual(
        Box((wing_length + 0.035, 0.060, 0.055)),
        origin=_radial_origin(wing_center, angle_2, 2.235),
        material=stainless,
        name="top_rail_2",
    )
    wing_2.visual(
        Box((wing_length + 0.035, 0.060, 0.055)),
        origin=_radial_origin(wing_center, angle_2, 0.385),
        material=stainless,
        name="bottom_rail_2",
    )
    wing_2.visual(
        Box((0.095, 0.070, 1.92)),
        origin=_radial_origin(0.1025, angle_2, 1.31),
        material=stainless,
        name="root_stile_2",
    )
    wing_2.visual(
        Box((0.045, 0.070, 1.94)),
        origin=_radial_origin(wing_outer, angle_2, 1.31),
        material=gasket,
        name="outer_seal_2",
    )
    wing_2.visual(
        Box((sweep_length, 0.012, 0.080)),
        origin=_radial_origin(sweep_center, angle_2, 0.325),
        material=gasket,
        name="sweep_seal_2",
    )
    wing_2.inertial = Inertial.from_geometry(
        Box((wing_outer, 0.08, 2.0)),
        mass=24.0,
        origin=Origin(xyz=(wing_center * math.cos(angle_2), wing_center * math.sin(angle_2), 1.31)),
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.97, length=2.36),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 1.31)),
    )

    for index, wing in enumerate((wing_0, wing_1, wing_2)):
        model.articulation(
            f"hub_to_wing_{index}",
            ArticulationType.FIXED,
            parent=rotor,
            child=wing,
            origin=Origin(),
        )

    model.articulation(
        "drum_to_wings",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("drum_housing")
    rotor = object_model.get_part("wing_assembly")
    wing = object_model.get_part("wing_0")
    joint = object_model.get_articulation("drum_to_wings")

    ctx.expect_within(
        wing,
        housing,
        axes="xy",
        margin=0.01,
        inner_elem="wing_panel_0",
        outer_elem="sealed_drum_shell",
        name="wing panel sits inside the sealed drum radius",
    )
    ctx.expect_within(
        rotor,
        housing,
        axes="xy",
        margin=0.0,
        inner_elem="central_post",
        outer_elem="lower_bearing",
        name="central post passes through lower bearing clearance",
    )
    ctx.expect_overlap(
        rotor,
        housing,
        axes="z",
        elem_a="central_post",
        elem_b="upper_bearing",
        min_overlap=0.06,
        name="upper bearing captures the vertical post",
    )

    ctx.expect_contact(
        rotor,
        housing,
        elem_a="lower_hub",
        elem_b="lower_bearing",
        contact_tol=0.001,
        name="lower hub is seated on the drum bearing",
    )
    ctx.expect_contact(
        wing,
        rotor,
        elem_a="root_stile_0",
        elem_b="central_post",
        contact_tol=0.001,
        name="wing root stile meets the central steel post",
    )

    closed_aabb = ctx.part_element_world_aabb(wing, elem="wing_panel_0")
    with ctx.pose({joint: math.pi / 2.0}):
        rotated_aabb = ctx.part_element_world_aabb(wing, elem="wing_panel_0")
        if closed_aabb is None or rotated_aabb is None:
            ctx.fail("wing panel pose is measurable", "Could not measure wing_panel_0 AABBs.")
        else:
            closed_center = (
                (closed_aabb[0][0] + closed_aabb[1][0]) * 0.5,
                (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5,
            )
            rotated_center = (
                (rotated_aabb[0][0] + rotated_aabb[1][0]) * 0.5,
                (rotated_aabb[0][1] + rotated_aabb[1][1]) * 0.5,
            )
            ctx.check(
                "continuous joint rotates wing assembly about the central axis",
                closed_center[0] > 0.45 and rotated_center[1] > 0.45,
                details=f"closed_center={closed_center}, rotated_center={rotated_center}",
            )

    return ctx.report()


object_model = build_object_model()
