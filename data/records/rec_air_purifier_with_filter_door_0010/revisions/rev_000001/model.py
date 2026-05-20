from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_air_purifier", assets=ASSETS)

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.92, 1.0))
    panel_white = model.material("panel_white", rgba=(0.90, 0.91, 0.89, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.10, 0.12, 0.14, 1.0))
    pedestal_gray = model.material("pedestal_gray", rgba=(0.55, 0.58, 0.60, 1.0))

    body = model.part("body")
    panel = model.part("front_panel")

    base_height = 0.04
    tower_height = 0.58
    top_height = 0.06
    tower_radius = 0.11
    tower_center_z = base_height + tower_height / 2.0
    top_center_z = base_height + tower_height + top_height / 2.0
    panel_center_z = 0.33

    body.visual(
        Cylinder(radius=tower_radius, length=tower_height),
        origin=Origin(xyz=(0.0, 0.0, tower_center_z)),
        material=shell_white,
        name="tower_shell",
    )
    body.visual(
        Cylinder(radius=0.125, length=base_height),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=pedestal_gray,
        name="base_ring",
    )
    body.visual(
        Cylinder(radius=0.105, length=top_height),
        origin=Origin(xyz=(0.0, 0.0, top_center_z)),
        material=shell_white,
        name="top_cap",
    )
    body.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.674)),
        material=graphite,
        name="top_vent",
    )
    body.visual(
        Box((0.028, 0.158, 0.414)),
        origin=Origin(xyz=(0.096, 0.0, panel_center_z)),
        material=shell_white,
        name="panel_seat",
    )
    body.visual(
        Box((0.012, 0.136, 0.366)),
        origin=Origin(xyz=(0.093, 0.0, panel_center_z)),
        material=graphite,
        name="filter_screen",
    )
    body.visual(
        Box((0.012, 0.058, 0.048)),
        origin=Origin(xyz=(0.098, 0.0, 0.545)),
        material=dark_glass,
        name="control_display",
    )
    body.visual(
        Box((0.008, 0.012, 0.340)),
        origin=Origin(xyz=(0.107, -0.075, panel_center_z)),
        material=graphite,
        name="hinge_jamb",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=tower_radius, length=tower_height),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, tower_center_z)),
    )

    panel.visual(
        Box((0.006, 0.152, 0.406)),
        origin=Origin(xyz=(0.0035, 0.076, 0.0)),
        material=panel_white,
        name="door_shell",
    )
    panel.visual(
        Box((0.004, 0.012, 0.130)),
        origin=Origin(xyz=(0.0065, 0.141, 0.0)),
        material=graphite,
        name="door_grip",
    )
    panel.visual(
        Box((0.004, 0.010, 0.340)),
        origin=Origin(xyz=(-0.001, 0.004, 0.0)),
        material=graphite,
        name="hinge_leaf",
    )
    panel.inertial = Inertial.from_geometry(
        Box((0.006, 0.152, 0.406)),
        mass=0.9,
        origin=Origin(xyz=(0.0035, 0.076, 0.0)),
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(xyz=(0.110, -0.076, panel_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-1.35,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    panel = object_model.get_part("front_panel")
    panel_hinge = object_model.get_articulation("panel_hinge")

    tower_shell = body.get_visual("tower_shell")
    panel_seat = body.get_visual("panel_seat")
    filter_screen = body.get_visual("filter_screen")
    hinge_jamb = body.get_visual("hinge_jamb")
    door_shell = panel.get_visual("door_shell")
    door_grip = panel.get_visual("door_grip")
    hinge_leaf = panel.get_visual("hinge_leaf")

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

    ctx.allow_overlap(
        panel,
        body,
        reason="the purifier door rides on a narrow hinge leaf nested in the body-side jamb",
    )
    ctx.expect_within(
        panel,
        body,
        axes="yz",
        inner_elem=door_shell,
        outer_elem=tower_shell,
        name="front access panel stays within the cylindrical tower silhouette",
    )
    ctx.expect_overlap(
        panel,
        body,
        axes="yz",
        min_overlap=0.05,
        elem_a=door_shell,
        elem_b=panel_seat,
        name="front access panel covers the service opening",
    )
    ctx.expect_gap(
        panel,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=door_shell,
        negative_elem=panel_seat,
        name="front access panel sits flush to the frame when closed",
    )
    ctx.expect_within(
        body,
        panel,
        axes="yz",
        inner_elem=filter_screen,
        outer_elem=door_shell,
        name="filter pack stays behind the access panel footprint",
    )
    ctx.expect_gap(
        panel,
        body,
        axis="x",
        min_gap=0.01,
        max_gap=0.02,
        positive_elem=door_shell,
        negative_elem=filter_screen,
        name="filter pack is recessed behind the closed panel",
    )
    ctx.expect_overlap(
        panel,
        body,
        axes="yz",
        min_overlap=0.006,
        elem_a=hinge_leaf,
        elem_b=hinge_jamb,
        name="front access panel stays captured by a vertical hinge spine",
    )
    ctx.expect_contact(
        panel,
        body,
        elem_a=hinge_leaf,
        elem_b=hinge_jamb,
        name="closed access panel remains physically seated on the hinge spine",
    )
    with ctx.pose({panel_hinge: -1.15}):
        ctx.expect_contact(
            panel,
            body,
            elem_a=hinge_leaf,
            elem_b=hinge_jamb,
            name="opened access panel remains attached at the hinge spine",
        )
        ctx.expect_gap(
            panel,
            body,
            axis="x",
            min_gap=0.10,
            positive_elem=door_grip,
            negative_elem=filter_screen,
            name="opened panel swings clear of the filter for service access",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
