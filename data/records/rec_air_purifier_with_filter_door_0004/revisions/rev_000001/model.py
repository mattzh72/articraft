from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_RADIUS = 0.090
BODY_SHELL_THICKNESS = 0.0045
PLINTH_RADIUS = 0.096
PLINTH_HEIGHT = 0.018
DRUM_HEIGHT = 0.142
DRUM_TOP = PLINTH_HEIGHT + DRUM_HEIGHT

PANEL_OPENING_WIDTH = 0.090
PANEL_OPENING_HEIGHT = 0.110
PANEL_OPENING_BOTTOM = 0.030
PANEL_CENTER_Z = PANEL_OPENING_BOTTOM + (PANEL_OPENING_HEIGHT * 0.5)
HINGE_BARREL_RADIUS = 0.005
HINGE_X = -(PANEL_OPENING_WIDTH * 0.5) - HINGE_BARREL_RADIUS
HINGE_Y = 0.096


def _save_mesh(geometry, filename: str):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _ring_shell(*, outer_radius: float, inner_radius: float, height: float):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=64)
    inner = CylinderGeometry(radius=inner_radius, height=height + 0.006, radial_segments=64)
    return boolean_difference(outer, inner)


def _build_drum_shell_mesh():
    shell = _ring_shell(
        outer_radius=BODY_RADIUS,
        inner_radius=BODY_RADIUS - BODY_SHELL_THICKNESS,
        height=DRUM_HEIGHT,
    ).translate(0.0, 0.0, PLINTH_HEIGHT + (DRUM_HEIGHT * 0.5))
    cutout = BoxGeometry(
        (
            PANEL_OPENING_WIDTH + 0.010,
            BODY_RADIUS * 1.28,
            PANEL_OPENING_HEIGHT + 0.012,
        )
    ).translate(0.0, BODY_RADIUS * 0.58, PANEL_CENTER_Z)
    return boolean_difference(shell, cutout)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_air_purifier", assets=ASSETS)

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.92, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.23, 0.24, 0.25, 1.0))
    filter_grey = model.material("filter_grey", rgba=(0.74, 0.78, 0.80, 1.0))
    filter_frame = model.material("filter_frame", rgba=(0.28, 0.29, 0.30, 1.0))
    soft_black = model.material("soft_black", rgba=(0.12, 0.13, 0.14, 1.0))

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=PLINTH_RADIUS, length=PLINTH_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=charcoal,
        name="plinth",
    )
    housing.visual(
        _save_mesh(_build_drum_shell_mesh(), "purifier_drum_shell.obj"),
        material=shell_white,
        name="drum_shell",
    )
    housing.visual(
        Box((PANEL_OPENING_WIDTH + 0.020, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.082, PANEL_OPENING_BOTTOM - 0.007)),
        material=shell_white,
        name="bottom_bezel",
    )
    housing.visual(
        Box((PANEL_OPENING_WIDTH + 0.020, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.082, PANEL_OPENING_BOTTOM + PANEL_OPENING_HEIGHT + 0.007)),
        material=shell_white,
        name="top_bezel",
    )
    housing.visual(
        Box((0.014, 0.016, PANEL_OPENING_HEIGHT + 0.020)),
        origin=Origin(xyz=(-(PANEL_OPENING_WIDTH * 0.5) - 0.007, 0.082, PANEL_CENTER_Z)),
        material=shell_white,
        name="left_bezel",
    )
    housing.visual(
        Box((0.014, 0.016, PANEL_OPENING_HEIGHT + 0.020)),
        origin=Origin(xyz=((PANEL_OPENING_WIDTH * 0.5) + 0.007, 0.082, PANEL_CENTER_Z)),
        material=shell_white,
        name="right_bezel",
    )
    housing.visual(
        Box((0.010, 0.002, PANEL_OPENING_HEIGHT - 0.014)),
        origin=Origin(
            xyz=((PANEL_OPENING_WIDTH * 0.5) + 0.003, HINGE_Y - 0.007, PANEL_CENTER_Z)
        ),
        material=dark_trim,
        name="latch_seat",
    )
    housing.inertial = Inertial.from_geometry(
        Cylinder(radius=PLINTH_RADIUS, length=DRUM_TOP),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, DRUM_TOP * 0.5)),
    )

    exhaust_cap = model.part("exhaust_cap")
    exhaust_cap.visual(
        Cylinder(radius=0.089, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_trim,
        name="cap_flange",
    )
    exhaust_cap.visual(
        Cylinder(radius=0.066, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_trim,
        name="cap_shell",
    )
    exhaust_cap.visual(
        Box((0.044, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=soft_black,
        name="grille_bar_x",
    )
    exhaust_cap.visual(
        Box((0.006, 0.044, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=soft_black,
        name="grille_bar_y",
    )
    exhaust_cap.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=soft_black,
        name="grille_hub",
    )
    exhaust_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.082, length=0.030),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((PANEL_OPENING_WIDTH, 0.004, PANEL_OPENING_HEIGHT)),
        origin=Origin(xyz=(HINGE_BARREL_RADIUS + (PANEL_OPENING_WIDTH * 0.5), -0.003, 0.0)),
        material=shell_white,
        name="door_skin",
    )
    access_panel.visual(
        Box((0.058, 0.005, 0.098)),
        origin=Origin(xyz=(0.056, 0.001, 0.0)),
        material=filter_frame,
        name="filter_frame",
    )
    access_panel.visual(
        Box((0.050, 0.004, 0.086)),
        origin=Origin(xyz=(0.056, 0.0015, 0.0)),
        material=filter_grey,
        name="filter_media",
    )
    for index in range(4):
        access_panel.visual(
            Box((0.0045, 0.0055, 0.086)),
            origin=Origin(xyz=(0.039 + (index * 0.011), 0.001, 0.0)),
            material=filter_frame,
            name=f"filter_pleat_{index}",
        )
    access_panel.visual(
        Box((0.007, 0.007, 0.026)),
        origin=Origin(xyz=(0.088, 0.0045, 0.0)),
        material=dark_trim,
        name="pull_tab",
    )
    access_panel.visual(
        Box((0.010, 0.005, 0.040)),
        origin=Origin(xyz=(0.091, -0.0055, 0.0)),
        material=dark_trim,
        name="latch_tongue",
    )
    access_panel.inertial = Inertial.from_geometry(
        Box((0.100, 0.040, 0.120)),
        mass=0.34,
        origin=Origin(xyz=(0.052, -0.012, 0.0)),
    )

    model.articulation(
        "housing_to_exhaust_cap",
        ArticulationType.FIXED,
        parent=housing,
        child=exhaust_cap,
        origin=Origin(xyz=(0.0, 0.0, DRUM_TOP)),
    )
    model.articulation(
        "housing_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=access_panel,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, PANEL_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    exhaust_cap = object_model.get_part("exhaust_cap")
    access_panel = object_model.get_part("access_panel")
    panel_hinge = object_model.get_articulation("housing_to_access_panel")

    drum_shell = housing.get_visual("drum_shell")
    latch_seat = housing.get_visual("latch_seat")
    cap_flange = exhaust_cap.get_visual("cap_flange")
    cap_shell = exhaust_cap.get_visual("cap_shell")
    door_skin = access_panel.get_visual("door_skin")
    filter_frame = access_panel.get_visual("filter_frame")
    filter_media = access_panel.get_visual("filter_media")
    latch_tongue = access_panel.get_visual("latch_tongue")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.08)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    ctx.expect_origin_distance(exhaust_cap, housing, axes="xy", max_dist=0.001)
    ctx.expect_within(exhaust_cap, housing, axes="xy", inner_elem=cap_shell, outer_elem=drum_shell)
    ctx.expect_contact(exhaust_cap, housing, elem_a=cap_flange, elem_b=drum_shell)

    ctx.expect_overlap(access_panel, housing, axes="xz", min_overlap=0.006, elem_a=door_skin)
    ctx.expect_contact(access_panel, housing, elem_a=latch_tongue, elem_b=latch_seat)
    ctx.expect_within(access_panel, housing, axes="xz", inner_elem=filter_frame, outer_elem=drum_shell)
    ctx.expect_within(access_panel, housing, axes="xz", inner_elem=filter_media, outer_elem=drum_shell)

    with ctx.pose({panel_hinge: 1.10}):
        ctx.expect_gap(
            access_panel,
            housing,
            axis="y",
            min_gap=0.015,
            positive_elem=filter_media,
            negative_elem=drum_shell,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
