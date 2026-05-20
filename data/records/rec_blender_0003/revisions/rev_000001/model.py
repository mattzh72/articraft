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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
    VentGrilleGeometry,
)

ASSETS = AssetContext.from_script(__file__)


def _grille_panel_geometry(
    panel_size,
    thickness,
    *,
    frame,
    slat_pitch,
    slat_width,
    slat_angle_deg,
    corner_radius,
    center=True,
):
    geom = VentGrilleGeometry(
        panel_size,
        frame=frame,
        face_thickness=thickness,
        duct_depth=max(0.0015, thickness * 0.75),
        duct_wall=max(0.001, min(frame * 0.45, thickness * 0.75)),
        slat_pitch=slat_pitch,
        slat_width=slat_width,
        slat_angle_deg=slat_angle_deg,
        corner_radius=corner_radius,
    )
    if not center:
        geom.translate(0.0, 0.0, thickness * 0.5)
    return geom


def _mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _section(width: float, depth: float, radius: float, z: float, *, y_shift: float = 0.0):
    return [(x, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=7)]


def _build_base_shell():
    return repair_loft(
        section_loft(
            [
                _section(0.238, 0.210, 0.024, 0.000, y_shift=0.000),
                _section(0.232, 0.198, 0.022, 0.038, y_shift=0.006),
                _section(0.220, 0.176, 0.020, 0.076, y_shift=0.015),
                _section(0.194, 0.146, 0.016, 0.106, y_shift=0.026),
            ]
        )
    )


def _build_drive_socket():
    return LatheGeometry.from_shell_profiles(
        [
            (0.034, 0.000),
            (0.039, 0.004),
            (0.039, 0.015),
            (0.032, 0.020),
        ],
        [
            (0.027, 0.000),
            (0.030, 0.004),
            (0.030, 0.013),
            (0.025, 0.018),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_jar_handle():
    return tube_from_spline_points(
        [
            (0.000, 0.000, 0.176),
            (0.015, 0.010, 0.168),
            (0.022, 0.019, 0.142),
            (0.026, 0.024, 0.100),
            (0.024, 0.021, 0.064),
            (0.014, 0.010, 0.040),
            (0.000, 0.000, 0.028),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _build_cap_socket():
    return LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.000),
            (0.025, 0.004),
            (0.025, 0.012),
            (0.020, 0.016),
        ],
        [
            (0.013, 0.000),
            (0.017, 0.004),
            (0.017, 0.010),
            (0.014, 0.014),
        ],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def _build_filler_plug():
    return LatheGeometry(
        [
            (0.000, 0.000),
            (0.016, 0.000),
            (0.014, 0.014),
            (0.000, 0.014),
        ],
        segments=44,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_countertop_blender", assets=ASSETS)

    cast_metal = model.material("cast_metal", rgba=(0.43, 0.45, 0.48, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.14, 0.15, 0.16, 1.0))
    button_black = model.material("button_black", rgba=(0.09, 0.09, 0.10, 1.0))
    dial_black = model.material("dial_black", rgba=(0.08, 0.08, 0.09, 1.0))
    vent_black = model.material("vent_black", rgba=(0.11, 0.12, 0.13, 1.0))
    jar_clear = model.material("jar_clear", rgba=(0.80, 0.90, 0.96, 0.34))
    jar_frame = model.material("jar_frame", rgba=(0.24, 0.25, 0.27, 1.0))
    lid_rubber = model.material("lid_rubber", rgba=(0.17, 0.18, 0.20, 1.0))
    cap_dark = model.material("cap_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(_mesh(_build_base_shell(), "blender_base_shell.obj"), material=cast_metal, name="base_shell")
    base.visual(
        Box((0.148, 0.118, 0.010)),
        origin=Origin(xyz=(0.000, 0.018, 0.102)),
        material=dark_panel,
        name="top_plate",
    )
    base.visual(
        _mesh(_build_drive_socket(), "blender_drive_socket.obj"),
        origin=Origin(xyz=(0.000, 0.018, 0.099)),
        material=dark_panel,
        name="drive_socket",
    )
    base.visual(
        Box((0.158, 0.010, 0.064)),
        origin=Origin(xyz=(0.000, -0.095, 0.047)),
        material=dark_panel,
        name="control_strip",
    )
    base.visual(
        Cylinder(radius=0.033, length=0.010),
        origin=Origin(xyz=(0.052, -0.095, 0.048), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="dial_recess",
    )
    for index, x_pos in enumerate((-0.048, -0.020, 0.008)):
        base.visual(
            Box((0.020, 0.004, 0.010)),
            origin=Origin(xyz=(x_pos, -0.094, 0.046)),
            material=button_black,
            name=f"button_{index + 1}",
        )
    base.visual(
        Box((0.020, 0.004, 0.004)),
        origin=Origin(xyz=(0.040, -0.094, 0.026)),
        material=button_black,
        name="pulse_button",
    )
    base.visual(
        _mesh(
            _grille_panel_geometry(
                panel_size=(0.116, 0.074),
                thickness=0.006,
                frame=0.008,
                slat_pitch=0.020,
                slat_width=0.008,
                slat_angle_deg=28.0,
                corner_radius=0.004,
                center=True,
            ),
            "blender_rear_vent.obj",
        ),
        origin=Origin(xyz=(0.000, 0.101, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=vent_black,
        name="rear_vent",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.238, 0.210, 0.108)),
        mass=6.4,
        origin=Origin(xyz=(0.000, 0.000, 0.054)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_knob",
    )
    dial.visual(
        Box((0.004, 0.003, 0.012)),
        origin=Origin(xyz=(0.015, -0.004, 0.000)),
        material=button_black,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.008),
        mass=0.14,
        origin=Origin(),
    )

    jar = model.part("jar")
    jar.visual(
        Box((0.112, 0.004, 0.196)),
        origin=Origin(xyz=(0.000, -0.054, 0.102)),
        material=jar_clear,
        name="front_wall",
    )
    jar.visual(
        Box((0.112, 0.004, 0.196)),
        origin=Origin(xyz=(0.000, 0.054, 0.102)),
        material=jar_clear,
        name="rear_wall",
    )
    jar.visual(
        Box((0.004, 0.104, 0.196)),
        origin=Origin(xyz=(-0.054, 0.000, 0.102)),
        material=jar_clear,
        name="left_wall",
    )
    jar.visual(
        Box((0.004, 0.104, 0.196)),
        origin=Origin(xyz=(0.054, 0.000, 0.102)),
        material=jar_clear,
        name="right_wall",
    )
    jar.visual(
        Box((0.108, 0.108, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=jar_clear,
        name="bottom_plate",
    )
    jar.visual(
        Box((0.120, 0.120, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.198)),
        material=jar_frame,
        name="top_frame",
    )
    jar.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.007)),
        material=jar_frame,
        name="drive_collar",
    )
    jar.visual(
        Box((0.024, 0.020, 0.020)),
        origin=Origin(xyz=(0.067, 0.000, 0.174)),
        material=jar_frame,
        name="handle_upper_mount",
    )
    jar.visual(
        Box((0.024, 0.020, 0.020)),
        origin=Origin(xyz=(0.067, 0.000, 0.030)),
        material=jar_frame,
        name="handle_lower_mount",
    )
    jar.visual(
        _mesh(_build_jar_handle(), "blender_jar_handle.obj"),
        origin=Origin(xyz=(0.078, 0.000, 0.000)),
        material=jar_frame,
        name="handle",
    )
    jar.inertial = Inertial.from_geometry(
        Box((0.138, 0.138, 0.214)),
        mass=1.6,
        origin=Origin(xyz=(0.000, 0.000, 0.100)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.126, 0.126, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=lid_rubber,
        name="lid_deck",
    )
    lid.visual(
        Box((0.126, 0.008, 0.014)),
        origin=Origin(xyz=(0.000, -0.059, 0.003)),
        material=lid_rubber,
        name="front_skirt",
    )
    lid.visual(
        Box((0.126, 0.008, 0.014)),
        origin=Origin(xyz=(0.000, 0.059, 0.003)),
        material=lid_rubber,
        name="rear_skirt",
    )
    lid.visual(
        Box((0.008, 0.110, 0.014)),
        origin=Origin(xyz=(-0.059, 0.000, 0.003)),
        material=lid_rubber,
        name="left_skirt",
    )
    lid.visual(
        Box((0.008, 0.110, 0.014)),
        origin=Origin(xyz=(0.059, 0.000, 0.003)),
        material=lid_rubber,
        name="right_skirt",
    )
    lid.visual(
        _mesh(_build_cap_socket(), "blender_cap_socket.obj"),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=lid_rubber,
        name="cap_socket",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.126, 0.126, 0.018)),
        mass=0.25,
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
    )

    filler_cap = model.part("filler_cap")
    filler_cap.visual(
        _mesh(_build_filler_plug(), "blender_filler_plug.obj"),
        material=cap_dark,
        name="plug",
    )
    filler_cap.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.017)),
        material=cap_dark,
        name="cap_disk",
    )
    filler_cap.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.023)),
        material=cap_dark,
        name="cap_grip",
    )
    filler_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.024),
        mass=0.05,
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
    )

    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.052, -0.094, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )
    model.articulation(
        "base_to_jar",
        ArticulationType.FIXED,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.000, 0.018, 0.119)),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.FIXED,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.000, 0.000, 0.204)),
    )
    model.articulation(
        "lid_to_filler_cap",
        ArticulationType.FIXED,
        parent=lid,
        child=filler_cap,
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    dial = object_model.get_part("dial")
    jar = object_model.get_part("jar")
    lid = object_model.get_part("lid")
    filler_cap = object_model.get_part("filler_cap")
    dial_joint = object_model.get_articulation("base_to_dial")

    control_strip = base.get_visual("control_strip")
    rear_vent = base.get_visual("rear_vent")
    dial_recess = base.get_visual("dial_recess")
    drive_socket = base.get_visual("drive_socket")
    top_plate = base.get_visual("top_plate")
    button_1 = base.get_visual("button_1")
    pulse_button = base.get_visual("pulse_button")
    dial_knob = dial.get_visual("dial_knob")
    drive_collar = jar.get_visual("drive_collar")
    bottom_plate = jar.get_visual("bottom_plate")
    right_wall = jar.get_visual("right_wall")
    handle = jar.get_visual("handle")
    top_frame = jar.get_visual("top_frame")
    lid_deck = lid.get_visual("lid_deck")
    front_skirt = lid.get_visual("front_skirt")
    cap_socket = lid.get_visual("cap_socket")
    plug = filler_cap.get_visual("plug")
    cap_disk = filler_cap.get_visual("cap_disk")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.allow_overlap(dial, base, reason="recessed rotary dial nests into the front control panel well")
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(jar, base, axes="xy", inner_elem=drive_collar, outer_elem=drive_socket)
    ctx.expect_contact(jar, base, elem_a=drive_collar, elem_b=drive_socket)
    ctx.expect_within(jar, lid, axes="xy", inner_elem=top_frame, outer_elem=lid_deck)
    ctx.expect_gap(
        lid,
        jar,
        axis="z",
        max_gap=0.002,
        max_penetration=0.003,
        positive_elem=front_skirt,
        negative_elem=top_frame,
    )
    ctx.expect_within(filler_cap, lid, axes="xy", inner_elem=plug, outer_elem=cap_socket)
    ctx.expect_gap(
        filler_cap,
        lid,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=cap_disk,
        negative_elem=lid_deck,
    )
    ctx.expect_gap(
        jar,
        jar,
        axis="x",
        min_gap=0.002,
        positive_elem=handle,
        negative_elem=right_wall,
    )
    ctx.expect_overlap(jar, jar, axes="yz", min_overlap=0.012, elem_a=handle, elem_b=right_wall)
    ctx.expect_within(base, base, axes="xz", inner_elem=button_1, outer_elem=control_strip)
    ctx.expect_within(base, base, axes="xz", inner_elem=pulse_button, outer_elem=control_strip)
    ctx.expect_within(dial, base, axes="xz", inner_elem=dial_knob, outer_elem=dial_recess)
    ctx.expect_overlap(dial, base, axes="yz", min_overlap=0.006, elem_a=dial_knob, elem_b=control_strip)
    ctx.expect_contact(dial, base, elem_a=dial_knob, elem_b=control_strip)
    ctx.expect_gap(
        base,
        base,
        axis="y",
        min_gap=0.180,
        positive_elem=rear_vent,
        negative_elem=control_strip,
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.200,
        positive_elem=lid_deck,
        negative_elem=top_plate,
    )
    with ctx.pose({dial_joint: math.pi / 2.0}):
        ctx.expect_within(dial, base, axes="xz", inner_elem=dial_knob, outer_elem=dial_recess)
        ctx.expect_overlap(dial, base, axes="yz", min_overlap=0.006, elem_a=dial_knob, elem_b=control_strip)
        ctx.expect_contact(dial, base, elem_a=dial_knob, elem_b=control_strip)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
