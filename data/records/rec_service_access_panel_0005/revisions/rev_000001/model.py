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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

OUTER_WIDTH = 0.620
OUTER_HEIGHT = 0.460
PLATE_THICKNESS = 0.012
OPENING_WIDTH = 0.422
OPENING_HEIGHT = 0.302
PANEL_WIDTH = 0.410
PANEL_HEIGHT = 0.290
HINGE_AXIS_X = -0.221
HINGE_Z = 0.021


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _hinge_knuckle_geometry(
    *,
    segments: list[tuple[float, float]],
    outer_radius: float,
    inner_radius: float,
    z_center: float,
) -> MeshGeometry:
    knuckles = MeshGeometry()
    for y_center, length in segments:
        outer = CylinderGeometry(radius=outer_radius, height=length, radial_segments=40)
        inner = CylinderGeometry(
            radius=inner_radius,
            height=length + 0.002,
            radial_segments=40,
        )
        barrel = boolean_difference(outer, inner)
        barrel.rotate_x(-math.pi / 2.0).translate(0.0, y_center, z_center)
        knuckles.merge(barrel)
    return knuckles


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_service_access_panel", assets=ASSETS)

    enclosure_matte = model.material("enclosure_matte", rgba=(0.20, 0.22, 0.24, 1.0))
    panel_satin = model.material("panel_satin", rgba=(0.58, 0.61, 0.64, 1.0))
    trim_satin = model.material("trim_satin", rgba=(0.70, 0.72, 0.74, 1.0))
    hardware_satin = model.material("hardware_satin", rgba=(0.46, 0.49, 0.52, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.07, 0.08, 0.09, 1.0))

    enclosure_face = model.part("enclosure_face")

    enclosure_face.visual(
        _save_mesh(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(OUTER_WIDTH, OUTER_HEIGHT, 0.022, corner_segments=10),
                [rounded_rect_profile(OPENING_WIDTH, OPENING_HEIGHT, 0.012, corner_segments=8)],
                height=PLATE_THICKNESS,
                center=False,
            ),
            "frame_plate.obj",
        ),
        material=enclosure_matte,
        name="frame_plate",
    )
    enclosure_face.visual(
        _save_mesh(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.458, 0.338, 0.016, corner_segments=8),
                [rounded_rect_profile(OPENING_WIDTH, OPENING_HEIGHT, 0.012, corner_segments=8)],
                height=0.0018,
                center=False,
            ),
            "opening_bezel.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0108)),
        material=trim_satin,
        name="opening_bezel",
    )

    enclosure_face.visual(
        Box((0.022, OUTER_HEIGHT, 0.030)),
        origin=Origin(xyz=(-0.299, 0.0, -0.015)),
        material=enclosure_matte,
        name="left_outer_flange",
    )
    enclosure_face.visual(
        Box((0.022, OUTER_HEIGHT, 0.030)),
        origin=Origin(xyz=(0.299, 0.0, -0.015)),
        material=enclosure_matte,
        name="right_outer_flange",
    )
    enclosure_face.visual(
        Box((OUTER_WIDTH - 0.044, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, 0.219, -0.015)),
        material=enclosure_matte,
        name="top_outer_flange",
    )
    enclosure_face.visual(
        Box((OUTER_WIDTH - 0.044, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, -0.219, -0.015)),
        material=enclosure_matte,
        name="bottom_outer_flange",
    )

    enclosure_face.visual(
        Box((0.010, 0.312, 0.020)),
        origin=Origin(xyz=(-0.216, 0.0, -0.010)),
        material=shadow_black,
        name="left_jamb",
    )
    enclosure_face.visual(
        Box((0.010, 0.312, 0.020)),
        origin=Origin(xyz=(0.216, 0.0, -0.010)),
        material=shadow_black,
        name="right_jamb",
    )
    enclosure_face.visual(
        Box((0.416, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.156, -0.010)),
        material=shadow_black,
        name="top_jamb",
    )
    enclosure_face.visual(
        Box((0.416, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.156, -0.010)),
        material=shadow_black,
        name="bottom_jamb",
    )

    enclosure_face.visual(
        Box((0.014, 0.316, 0.005)),
        origin=Origin(xyz=(HINGE_AXIS_X - 0.007, 0.0, 0.0145)),
        material=hardware_satin,
        name="frame_hinge_leaf",
    )
    enclosure_face.visual(
        _save_mesh(
            _hinge_knuckle_geometry(
                segments=[(0.126, 0.042), (0.0, 0.042), (-0.126, 0.042)],
                outer_radius=0.0085,
                inner_radius=0.0052,
                z_center=HINGE_Z,
            ),
            "frame_knuckles.obj",
        ),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, 0.0)),
        material=hardware_satin,
        name="frame_knuckles",
    )

    enclosure_face.visual(
        Box((0.004, 0.034, 0.018)),
        origin=Origin(xyz=(0.209, 0.084, -0.009)),
        material=trim_satin,
        name="upper_striker",
    )
    enclosure_face.visual(
        Box((0.004, 0.034, 0.018)),
        origin=Origin(xyz=(0.209, -0.084, -0.009)),
        material=trim_satin,
        name="lower_striker",
    )

    for index, (x_pos, y_pos) in enumerate(
        [
            (-0.252, 0.168),
            (0.252, 0.168),
            (-0.252, -0.168),
            (0.252, -0.168),
            (0.0, 0.186),
            (0.0, -0.186),
        ]
    ):
        enclosure_face.visual(
            Cylinder(radius=0.005, length=0.0018),
            origin=Origin(xyz=(x_pos, y_pos, 0.0111)),
            material=hardware_satin,
            name=f"frame_fastener_{index}",
        )

    enclosure_face.inertial = Inertial.from_geometry(
        Box((OUTER_WIDTH, OUTER_HEIGHT, 0.050)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
    )

    hinge_pin = model.part("hinge_pin")
    hinge_pin.visual(
        Cylinder(radius=0.0035, length=0.330),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_satin,
        name="pin_shaft",
    )
    hinge_pin.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, 0.149, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_satin,
        name="pin_cap_top",
    )
    hinge_pin.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, -0.149, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_satin,
        name="pin_cap_bottom",
    )
    hinge_pin.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0035, length=0.330),
        mass=0.10,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((PANEL_WIDTH, PANEL_HEIGHT, 0.010)),
        origin=Origin(xyz=(0.219, 0.0, -0.015)),
        material=panel_satin,
        name="panel_skin",
    )
    access_panel.visual(
        Box((0.026, 0.268, 0.032)),
        origin=Origin(xyz=(0.027, 0.0, -0.015)),
        material=panel_satin,
        name="hinge_stile",
    )
    access_panel.visual(
        Box((0.028, 0.246, 0.016)),
        origin=Origin(xyz=(0.407, 0.0, -0.028)),
        material=panel_satin,
        name="latch_stile",
    )
    access_panel.visual(
        Box((0.378, 0.024, 0.016)),
        origin=Origin(xyz=(0.219, 0.133, -0.028)),
        material=panel_satin,
        name="top_stiffener",
    )
    access_panel.visual(
        Box((0.378, 0.024, 0.016)),
        origin=Origin(xyz=(0.219, -0.133, -0.028)),
        material=panel_satin,
        name="bottom_stiffener",
    )
    access_panel.visual(
        Box((0.014, 0.280, 0.008)),
        origin=Origin(xyz=(0.018, 0.0, -0.003)),
        material=hardware_satin,
        name="panel_hinge_leaf",
    )
    access_panel.visual(
        _save_mesh(
            _hinge_knuckle_geometry(
                segments=[(0.063, 0.042), (-0.063, 0.042)],
                outer_radius=0.0085,
                inner_radius=0.0052,
                z_center=0.0,
            ),
            "panel_knuckles.obj",
        ),
        origin=Origin(xyz=(0.0025, 0.0, 0.0)),
        material=hardware_satin,
        name="panel_knuckles",
    )
    access_panel.visual(
        Cylinder(radius=0.008, length=0.0022),
        origin=Origin(xyz=(0.392, 0.084, -0.0101)),
        material=hardware_satin,
        name="latch_upper",
    )
    access_panel.visual(
        Cylinder(radius=0.008, length=0.0022),
        origin=Origin(xyz=(0.392, -0.084, -0.0101)),
        material=hardware_satin,
        name="latch_lower",
    )
    access_panel.visual(
        Box((0.012, 0.0026, 0.0012)),
        origin=Origin(xyz=(0.392, 0.084, -0.0096)),
        material=shadow_black,
        name="latch_upper_slot",
    )
    access_panel.visual(
        Box((0.012, 0.0026, 0.0012)),
        origin=Origin(xyz=(0.392, -0.084, -0.0096)),
        material=shadow_black,
        name="latch_lower_slot",
    )
    access_panel.inertial = Inertial.from_geometry(
        Box((0.432, 0.302, 0.030)),
        mass=1.6,
        origin=Origin(xyz=(0.216, 0.0, -0.025)),
    )

    model.articulation(
        "enclosure_to_hinge_pin",
        ArticulationType.FIXED,
        parent=enclosure_face,
        child=hinge_pin,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_Z)),
    )
    model.articulation(
        "hinge_pin_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=hinge_pin,
        child=access_panel,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    enclosure_face = object_model.get_part("enclosure_face")
    hinge_pin = object_model.get_part("hinge_pin")
    access_panel = object_model.get_part("access_panel")

    hinge_pin_joint = object_model.get_articulation("enclosure_to_hinge_pin")
    panel_hinge = object_model.get_articulation("hinge_pin_to_access_panel")

    frame_plate = enclosure_face.get_visual("frame_plate")
    left_jamb = enclosure_face.get_visual("left_jamb")
    right_jamb = enclosure_face.get_visual("right_jamb")
    top_jamb = enclosure_face.get_visual("top_jamb")
    bottom_jamb = enclosure_face.get_visual("bottom_jamb")
    frame_knuckles = enclosure_face.get_visual("frame_knuckles")
    upper_striker = enclosure_face.get_visual("upper_striker")
    lower_striker = enclosure_face.get_visual("lower_striker")

    pin_shaft = hinge_pin.get_visual("pin_shaft")

    panel_skin = access_panel.get_visual("panel_skin")
    panel_knuckles = access_panel.get_visual("panel_knuckles")
    latch_stile = access_panel.get_visual("latch_stile")
    latch_upper = access_panel.get_visual("latch_upper")

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
    ctx.allow_overlap(
        access_panel,
        hinge_pin,
        elem_a=panel_knuckles,
        elem_b=pin_shaft,
        reason="The hinge pin is intentionally captured inside the bored panel knuckles as a real pivot.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_overlap(
        enclosure_face,
        hinge_pin,
        axes="yz",
        elem_a=frame_knuckles,
        elem_b=pin_shaft,
        min_overlap=0.006,
        name="frame_knuckles_align_to_pin",
    )
    ctx.expect_overlap(
        access_panel,
        hinge_pin,
        axes="yz",
        elem_a=panel_knuckles,
        elem_b=pin_shaft,
        min_overlap=0.006,
        name="panel_knuckles_align_to_pin",
    )
    ctx.expect_gap(
        access_panel,
        enclosure_face,
        axis="x",
        positive_elem=panel_skin,
        negative_elem=left_jamb,
        min_gap=0.004,
        max_gap=0.008,
        name="left_seam_gap",
    )
    ctx.expect_gap(
        enclosure_face,
        access_panel,
        axis="x",
        positive_elem=right_jamb,
        negative_elem=panel_skin,
        min_gap=0.004,
        max_gap=0.0082,
        name="right_seam_gap",
    )
    ctx.expect_gap(
        enclosure_face,
        access_panel,
        axis="y",
        positive_elem=top_jamb,
        negative_elem=panel_skin,
        min_gap=0.004,
        max_gap=0.008,
        name="top_seam_gap",
    )
    ctx.expect_gap(
        access_panel,
        enclosure_face,
        axis="y",
        positive_elem=panel_skin,
        negative_elem=bottom_jamb,
        min_gap=0.004,
        max_gap=0.008,
        name="bottom_seam_gap",
    )
    ctx.expect_gap(
        access_panel,
        enclosure_face,
        axis="z",
        positive_elem=panel_skin,
        negative_elem=left_jamb,
        min_gap=0.0008,
        max_gap=0.003,
        name="panel_reveal_offset",
    )
    ctx.expect_gap(
        enclosure_face,
        access_panel,
        axis="x",
        positive_elem=upper_striker,
        negative_elem=latch_stile,
        min_gap=0.001,
        max_gap=0.0072,
        name="upper_latch_clearance",
    )
    ctx.expect_gap(
        enclosure_face,
        access_panel,
        axis="x",
        positive_elem=lower_striker,
        negative_elem=latch_stile,
        min_gap=0.001,
        max_gap=0.0072,
        name="lower_latch_clearance",
    )
    ctx.expect_overlap(
        access_panel,
        enclosure_face,
        axes="xy",
        elem_a=panel_skin,
        elem_b=frame_plate,
        min_overlap=0.28,
        name="panel_covers_service_opening",
    )

    with ctx.pose({panel_hinge: math.radians(82.0)}):
        ctx.expect_overlap(
            access_panel,
            hinge_pin,
            axes="yz",
            elem_a=panel_knuckles,
            elem_b=pin_shaft,
            min_overlap=0.006,
            name="open_pose_pin_alignment",
        )
        ctx.expect_gap(
            access_panel,
            enclosure_face,
            axis="z",
            positive_elem=latch_upper,
            negative_elem=frame_plate,
            min_gap=0.18,
            name="open_pose_latch_edge_clears_frame",
        )

    hinge_pin_pos = ctx.part_world_position(hinge_pin)
    panel_rest_pos = ctx.part_world_position(access_panel)
    assert hinge_pin_pos is not None
    assert panel_rest_pos is not None
    assert hinge_pin_joint.articulation_type is ArticulationType.FIXED

    with ctx.pose({panel_hinge: math.radians(82.0)}):
        panel_open_pos = ctx.part_world_position(access_panel)
        assert panel_open_pos is not None
        assert abs(panel_open_pos[0] - hinge_pin_pos[0]) < 1e-6
        assert abs(panel_open_pos[2] - hinge_pin_pos[2]) < 1e-6

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
