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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BASE_RADIUS = 0.145
BASE_DISC_THICK = 0.022
BASE_PLINTH_RADIUS = 0.110
BASE_PLINTH_THICK = 0.012
BEARING_RING_OUTER = 0.061
BEARING_RING_INNER = 0.038
BEARING_RING_THICK = 0.010
PIVOT_Z = BASE_DISC_THICK + BASE_PLINTH_THICK + BEARING_RING_THICK

FLANGE_RADIUS = 0.050
FLANGE_THICK = 0.006
COLLAR_RADIUS = 0.030
COLLAR_THICK = 0.028

NECK_SIZE = (0.082, 0.064, 0.042)
NECK_CENTER_Z = 0.035

SHELL_WIDTH = 0.132
SHELL_DEPTH = 0.112
SHELL_CORNER = 0.024
SHELL_HEIGHT = 0.778
SHELL_BASE_Z = 0.052

INNER_WIDTH = 0.102
INNER_DEPTH = 0.078
INNER_CORNER = 0.018
INNER_HEIGHT = 0.720
INNER_SHIFT_Y = 0.019
INNER_BASE_Z = 0.028

TOP_CAP_SIZE = (0.094, 0.070, 0.010)
TOP_CAP_CENTER_Z = SHELL_BASE_Z + SHELL_HEIGHT - (TOP_CAP_SIZE[2] * 0.5)

GRILLE_WIDTH = 0.112
GRILLE_HEIGHT = 0.748
GRILLE_THICK = 0.006
GRILLE_SLOT_COUNT = 10
GRILLE_SLOT_WIDTH = 0.0068
GRILLE_SLOT_HEIGHT = 0.708
GRILLE_SLOT_RADIUS = 0.0030
GRILLE_CENTER_Y = (SHELL_DEPTH * 0.5) - (GRILLE_THICK * 0.5) + 0.0005
GRILLE_CENTER_Z = SHELL_BASE_Z + (SHELL_HEIGHT * 0.5)

KNOB_BOSS_RADIUS = 0.020
KNOB_BOSS_THICK = 0.014
KNOB_RADIUS = 0.029
KNOB_THICK = 0.018
KNOB_CAP_RADIUS = 0.012
KNOB_CAP_THICK = 0.003
KNOB_MOUNT_Z = SHELL_BASE_Z + SHELL_HEIGHT - 0.0005


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_bearing_ring_mesh():
    outer = CylinderGeometry(radius=BEARING_RING_OUTER, height=BEARING_RING_THICK, radial_segments=56)
    inner = CylinderGeometry(
        radius=BEARING_RING_INNER,
        height=BEARING_RING_THICK + 0.002,
        radial_segments=56,
    )
    return boolean_difference(outer, inner)


def _build_column_shell_mesh():
    outer = ExtrudeGeometry.from_z0(
        rounded_rect_profile(SHELL_WIDTH, SHELL_DEPTH, SHELL_CORNER, corner_segments=10),
        SHELL_HEIGHT,
        cap=True,
        closed=True,
    )
    inner = ExtrudeGeometry.from_z0(
        rounded_rect_profile(INNER_WIDTH, INNER_DEPTH, INNER_CORNER, corner_segments=8),
        INNER_HEIGHT,
        cap=True,
        closed=True,
    ).translate(0.0, INNER_SHIFT_Y, INNER_BASE_Z)
    return boolean_difference(outer, inner)


def _build_grille_mesh():
    outer_profile = rounded_rect_profile(GRILLE_WIDTH, GRILLE_HEIGHT, 0.015, corner_segments=8)
    hole_profiles: list[list[tuple[float, float]]] = []
    gap = (GRILLE_WIDTH - (GRILLE_SLOT_COUNT * GRILLE_SLOT_WIDTH)) / (GRILLE_SLOT_COUNT + 1)
    left_edge = -0.5 * GRILLE_WIDTH + gap + (GRILLE_SLOT_WIDTH * 0.5)
    slot_profile = rounded_rect_profile(
        GRILLE_SLOT_WIDTH,
        GRILLE_SLOT_HEIGHT,
        GRILLE_SLOT_RADIUS,
        corner_segments=6,
    )
    for index in range(GRILLE_SLOT_COUNT):
        x_pos = left_edge + index * (GRILLE_SLOT_WIDTH + gap)
        hole_profiles.append(_shift_profile(slot_profile, dx=x_pos))
    return ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=GRILLE_THICK,
        center=True,
        closed=True,
    ).rotate_x(math.pi * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_box_fan", assets=ASSETS)

    shell_white = model.material("shell_white", rgba=(0.91, 0.92, 0.93, 1.0))
    shadow_charcoal = model.material("shadow_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.22, 0.23, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_DISC_THICK),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_THICK * 0.5)),
        material=shell_white,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=BASE_PLINTH_RADIUS, length=BASE_PLINTH_THICK),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_THICK + (BASE_PLINTH_THICK * 0.5))),
        material=shell_white,
        name="base_plinth",
    )
    base.visual(
        _save_mesh("tower_fan_bearing_ring.obj", _build_bearing_ring_mesh()),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_THICK + BASE_PLINTH_THICK + (BEARING_RING_THICK * 0.5))),
        material=trim_gray,
        name="bearing_ring",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=PIVOT_Z),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z * 0.5)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_THICK),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_THICK * 0.5)),
        material=trim_gray,
        name="pivot_flange",
    )
    column.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_THICK),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=trim_gray,
        name="pivot_collar",
    )
    column.visual(
        Box(NECK_SIZE),
        origin=Origin(xyz=(0.0, 0.0, NECK_CENTER_Z)),
        material=shell_white,
        name="column_neck",
    )
    column.visual(
        _save_mesh("tower_fan_column_shell.obj", _build_column_shell_mesh()),
        origin=Origin(xyz=(0.0, 0.0, SHELL_BASE_Z)),
        material=shell_white,
        name="tower_shell",
    )
    column.visual(
        Box((0.094, 0.060, 0.730)),
        origin=Origin(xyz=(0.0, 0.012, GRILLE_CENTER_Z)),
        material=shadow_charcoal,
        name="air_channel",
    )
    column.visual(
        Box(TOP_CAP_SIZE),
        origin=Origin(xyz=(0.0, 0.0, TOP_CAP_CENTER_Z)),
        material=shell_white,
        name="top_cap",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.16, 0.14, 0.86)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )

    grille = model.part("grille")
    grille.visual(
        _save_mesh("tower_fan_front_grille.obj", _build_grille_mesh()),
        material=shadow_charcoal,
        name="front_grille",
    )
    grille.inertial = Inertial.from_geometry(
        Box((GRILLE_WIDTH, GRILLE_THICK, GRILLE_HEIGHT)),
        mass=0.25,
        origin=Origin(),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=KNOB_BOSS_RADIUS, length=KNOB_BOSS_THICK),
        origin=Origin(xyz=(0.0, 0.0, KNOB_BOSS_THICK * 0.5)),
        material=trim_gray,
        name="knob_boss",
    )
    knob.visual(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_THICK),
        origin=Origin(xyz=(0.0, 0.0, KNOB_BOSS_THICK + (KNOB_THICK * 0.5))),
        material=knob_dark,
        name="speed_knob",
    )
    knob.visual(
        Cylinder(radius=KNOB_CAP_RADIUS, length=KNOB_CAP_THICK),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                KNOB_BOSS_THICK + KNOB_THICK + (KNOB_CAP_THICK * 0.5),
            )
        ),
        material=trim_gray,
        name="knob_cap",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_BOSS_THICK + KNOB_THICK + KNOB_CAP_THICK),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.REVOLUTE,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-0.8, upper=0.8),
    )
    model.articulation(
        "column_to_grille",
        ArticulationType.FIXED,
        parent=column,
        child=grille,
        origin=Origin(xyz=(0.0, GRILLE_CENTER_Y, GRILLE_CENTER_Z)),
    )
    model.articulation(
        "column_to_knob",
        ArticulationType.FIXED,
        parent=column,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, KNOB_MOUNT_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    grille = object_model.get_part("grille")
    knob = object_model.get_part("knob")
    oscillation = object_model.get_articulation("base_to_column")

    bearing_ring = base.get_visual("bearing_ring")
    pivot_flange = column.get_visual("pivot_flange")
    pivot_collar = column.get_visual("pivot_collar")
    column_neck = column.get_visual("column_neck")
    tower_shell = column.get_visual("tower_shell")
    top_cap = column.get_visual("top_cap")
    front_grille = grille.get_visual("front_grille")
    knob_boss = knob.get_visual("knob_boss")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
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
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_contact(column, base, elem_a=pivot_flange, elem_b=bearing_ring)
    ctx.expect_within(column, base, axes="xy")
    ctx.expect_within(column, base, axes="xy", inner_elem=pivot_collar, outer_elem=bearing_ring)
    ctx.expect_origin_distance(column, base, axes="xy", max_dist=0.001)
    ctx.expect_gap(
        column,
        base,
        axis="z",
        min_gap=0.80,
        positive_elem=top_cap,
        negative_elem=bearing_ring,
        name="tower_column_reads_tall_above_base",
    )

    ctx.expect_contact(grille, column, elem_a=front_grille, elem_b=tower_shell)
    ctx.expect_within(grille, column, axes="xz")
    ctx.expect_origin_distance(grille, column, axes="x", max_dist=0.001)
    ctx.expect_gap(
        column,
        grille,
        axis="z",
        max_gap=0.020,
        max_penetration=0.0,
        positive_elem=top_cap,
        negative_elem=front_grille,
        name="grille_reaches_near_top_cap",
    )
    ctx.expect_gap(
        grille,
        column,
        axis="z",
        max_gap=0.020,
        max_penetration=0.0,
        positive_elem=front_grille,
        negative_elem=column_neck,
        name="grille_starts_low_on_column",
    )

    ctx.expect_contact(knob, column, elem_a=knob_boss, elem_b=top_cap)
    ctx.expect_within(knob, column, axes="xy")
    ctx.expect_origin_distance(knob, column, axes="xy", max_dist=0.001)

    with ctx.pose({oscillation: 0.65}):
        ctx.expect_contact(column, base, elem_a=pivot_flange, elem_b=bearing_ring)
        ctx.expect_within(column, base, axes="xy")
        ctx.expect_within(column, base, axes="xy", inner_elem=pivot_collar, outer_elem=bearing_ring)
        ctx.expect_origin_distance(column, base, axes="xy", max_dist=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
