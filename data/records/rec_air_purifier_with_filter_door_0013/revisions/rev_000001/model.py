from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_air_purifier", assets=ASSETS)

    width = 0.34
    height = 0.62
    depth = 0.105
    rear_depth = 0.010
    shell_depth = depth - rear_depth
    side_width = 0.014
    top_height = 0.055
    bottom_height = 0.045

    housing_white = model.material("housing_white", rgba=(0.94, 0.95, 0.96, 1.0))
    panel_white = model.material("panel_white", rgba=(0.97, 0.97, 0.98, 1.0))
    vent_gray = model.material("vent_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.76, 0.79, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        Box((rear_depth, width, height)),
        origin=Origin(xyz=(rear_depth / 2.0, 0.0, 0.0)),
        material=housing_white,
        name="back_plate",
    )
    body.visual(
        Box((shell_depth, side_width, height)),
        origin=Origin(xyz=(rear_depth + shell_depth / 2.0, -width / 2.0 + side_width / 2.0, 0.0)),
        material=housing_white,
        name="left_side",
    )
    body.visual(
        Box((shell_depth, side_width, height)),
        origin=Origin(xyz=(rear_depth + shell_depth / 2.0, width / 2.0 - side_width / 2.0, 0.0)),
        material=housing_white,
        name="right_side",
    )
    body.visual(
        Box((shell_depth, width - 2.0 * side_width, top_height)),
        origin=Origin(xyz=(rear_depth + shell_depth / 2.0, 0.0, height / 2.0 - top_height / 2.0)),
        material=housing_white,
        name="top_cap",
    )
    body.visual(
        Box((shell_depth, width - 2.0 * side_width, bottom_height)),
        origin=Origin(xyz=(rear_depth + shell_depth / 2.0, 0.0, -height / 2.0 + bottom_height / 2.0)),
        material=housing_white,
        name="bottom_cap",
    )
    body.visual(
        Box((0.020, 0.272, 0.430)),
        origin=Origin(xyz=(0.020, 0.0, -0.015)),
        material=filter_gray,
        name="filter_core",
    )
    body.visual(
        Box((0.050, 0.210, 0.008)),
        origin=Origin(xyz=(0.055, 0.0, height / 2.0 - 0.001)),
        material=accent_gray,
        name="outlet_deck",
    )
    for index, y in enumerate((-0.060, -0.030, 0.000, 0.030, 0.060), start=1):
        body.visual(
            Box((0.042, 0.012, 0.010)),
            origin=Origin(xyz=(0.055, y, height / 2.0 + 0.0005)),
            material=vent_gray,
            name=f"outlet_fin_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((depth, width, height)),
        mass=4.6,
        origin=Origin(xyz=(depth / 2.0, 0.0, 0.0)),
    )

    panel = model.part("front_panel")
    panel_width = 0.314
    panel_height = 0.530
    panel_thickness = 0.006
    panel.visual(
        Box((panel_thickness, panel_width, panel_height)),
        origin=Origin(xyz=(0.006, 0.0, -panel_height / 2.0)),
        material=panel_white,
        name="panel_shell",
    )
    panel.visual(
        Box((0.003, 0.250, 0.020)),
        origin=Origin(xyz=(0.0095, 0.0, -0.360)),
        material=vent_gray,
        name="grille_bar_1",
    )
    panel.visual(
        Box((0.003, 0.250, 0.020)),
        origin=Origin(xyz=(0.0095, 0.0, -0.395)),
        material=vent_gray,
        name="grille_bar_2",
    )
    panel.visual(
        Box((0.003, 0.250, 0.020)),
        origin=Origin(xyz=(0.0095, 0.0, -0.430)),
        material=vent_gray,
        name="grille_bar_3",
    )
    panel.visual(
        Box((0.003, 0.250, 0.020)),
        origin=Origin(xyz=(0.0095, 0.0, -0.465)),
        material=vent_gray,
        name="grille_bar_4",
    )
    panel.visual(
        Box((0.002, 0.075, 0.012)),
        origin=Origin(xyz=(0.0090, 0.0, -0.115)),
        material=accent_gray,
        name="status_strip",
    )
    panel.visual(
        Box((0.012, 0.120, 0.020)),
        origin=Origin(xyz=(0.010, 0.0, -0.520)),
        material=accent_gray,
        name="bottom_pull",
    )
    panel.inertial = Inertial.from_geometry(
        Box((0.018, panel_width, panel_height)),
        mass=1.1,
        origin=Origin(xyz=(0.009, 0.0, -panel_height / 2.0)),
    )

    model.articulation(
        "front_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(xyz=(0.102, 0.0, 0.255)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-1.18,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    panel = object_model.get_part("front_panel")
    hinge = object_model.get_articulation("front_panel_hinge")
    panel_shell = panel.get_visual("panel_shell")
    bottom_pull = panel.get_visual("bottom_pull")
    top_cap = body.get_visual("top_cap")
    bottom_cap = body.get_visual("bottom_cap")
    filter_core = body.get_visual("filter_core")
    outlet_deck = body.get_visual("outlet_deck")

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
    ctx.expect_within(panel, body, axes="yz", inner_elem=panel_shell)
    ctx.expect_within(body, panel, axes="yz", inner_elem=filter_core, outer_elem=panel_shell)
    ctx.expect_overlap(panel, body, axes="yz", min_overlap=0.12, elem_a=panel_shell)
    ctx.expect_gap(
        panel,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=panel_shell,
    )
    ctx.expect_gap(
        body,
        panel,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=top_cap,
        negative_elem=panel_shell,
    )
    ctx.expect_gap(
        body,
        panel,
        axis="z",
        min_gap=0.040,
        positive_elem=outlet_deck,
        negative_elem=panel_shell,
    )
    with ctx.pose({hinge: -1.05}):
        ctx.expect_gap(
            panel,
            body,
            axis="x",
            min_gap=0.140,
            positive_elem=bottom_pull,
        )
        ctx.expect_gap(
            panel,
            body,
            axis="z",
            min_gap=0.240,
            positive_elem=bottom_pull,
            negative_elem=bottom_cap,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
