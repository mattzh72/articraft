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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_roof_vent_tower", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.30, 0.34, 0.36, 1.0))
    darker_paint = model.material("darker_paint", rgba=(0.22, 0.25, 0.27, 1.0))
    molded_polymer = model.material("molded_polymer", rgba=(0.14, 0.15, 0.16, 1.0))
    fastener_zinc = model.material("fastener_zinc", rgba=(0.71, 0.73, 0.76, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.09, 0.10, 1.0))

    def add_bolt(
        part,
        name: str,
        xyz: tuple[float, float, float],
        *,
        axis: str = "z",
        radius: float = 0.005,
        length: float = 0.004,
    ) -> None:
        rpy = {
            "x": (0.0, math.pi / 2.0, 0.0),
            "y": (math.pi / 2.0, 0.0, 0.0),
            "z": (0.0, 0.0, 0.0),
        }[axis]
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=fastener_zinc,
            name=name,
        )

    tower = model.part("tower")

    tower.visual(
        Box((0.48, 0.42, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=painted_steel,
        name="roof_flashing",
    )

    curb_height = 0.065
    curb_z = 0.010 + curb_height / 2.0
    tower.visual(
        Box((0.38, 0.015, curb_height)),
        origin=Origin(xyz=(0.0, 0.1625, curb_z)),
        material=painted_steel,
        name="curb_front",
    )
    tower.visual(
        Box((0.38, 0.015, curb_height)),
        origin=Origin(xyz=(0.0, -0.1625, curb_z)),
        material=painted_steel,
        name="curb_back",
    )
    tower.visual(
        Box((0.015, 0.31, curb_height)),
        origin=Origin(xyz=(-0.1825, 0.0, curb_z)),
        material=painted_steel,
        name="curb_left",
    )
    tower.visual(
        Box((0.015, 0.31, curb_height)),
        origin=Origin(xyz=(0.1825, 0.0, curb_z)),
        material=painted_steel,
        name="curb_right",
    )

    body_z = 0.225
    body_h = 0.31
    tower.visual(
        Box((0.34, 0.012, body_h)),
        origin=Origin(xyz=(0.0, -0.139, body_z)),
        material=painted_steel,
        name="back_wall",
    )
    tower.visual(
        Box((0.012, 0.278, body_h)),
        origin=Origin(xyz=(-0.164, 0.0, body_z)),
        material=painted_steel,
        name="left_wall",
    )
    tower.visual(
        Box((0.012, 0.278, body_h)),
        origin=Origin(xyz=(0.164, 0.0, body_z)),
        material=painted_steel,
        name="right_wall",
    )
    tower.visual(
        Box((0.34, 0.012, 0.135)),
        origin=Origin(xyz=(0.0, 0.139, 0.1375)),
        material=painted_steel,
        name="front_lower_panel",
    )
    tower.visual(
        Box((0.055, 0.012, 0.12)),
        origin=Origin(xyz=(-0.1425, 0.139, 0.265)),
        material=painted_steel,
        name="front_left_stile",
    )
    tower.visual(
        Box((0.055, 0.012, 0.12)),
        origin=Origin(xyz=(0.1425, 0.139, 0.265)),
        material=painted_steel,
        name="front_right_stile",
    )
    tower.visual(
        Box((0.34, 0.012, 0.015)),
        origin=Origin(xyz=(0.0, 0.139, 0.3725)),
        material=painted_steel,
        name="front_header",
    )
    tower.visual(
        Box((0.055, 0.012, 0.030)),
        origin=Origin(xyz=(-0.1425, 0.139, 0.350)),
        material=darker_paint,
        name="front_left_cap",
    )
    tower.visual(
        Box((0.055, 0.012, 0.030)),
        origin=Origin(xyz=(0.1425, 0.139, 0.350)),
        material=darker_paint,
        name="front_right_cap",
    )
    tower.visual(
        Box((0.34, 0.16, 0.010)),
        origin=Origin(xyz=(0.0, -0.045, 0.375)),
        material=painted_steel,
        name="rear_roof_deck",
    )

    for name, xyz in (
        ("front_left_post", (-0.159, 0.134, body_z)),
        ("front_right_post", (0.159, 0.134, body_z)),
        ("rear_left_post", (-0.159, -0.134, body_z)),
        ("rear_right_post", (0.159, -0.134, body_z)),
    ):
        tower.visual(
            Box((0.022, 0.022, body_h)),
            origin=Origin(xyz=xyz),
            material=darker_paint,
            name=name,
        )

    tower.visual(
        Box((0.27, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.145, 0.190)),
        material=darker_paint,
        name="opening_frame_sill",
    )
    tower.visual(
        Box((0.27, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.145, 0.340)),
        material=darker_paint,
        name="opening_frame_header",
    )
    tower.visual(
        Box((0.030, 0.024, 0.12)),
        origin=Origin(xyz=(-0.130, 0.145, 0.265)),
        material=darker_paint,
        name="opening_left_frame",
    )
    tower.visual(
        Box((0.030, 0.024, 0.12)),
        origin=Origin(xyz=(0.130, 0.145, 0.265)),
        material=darker_paint,
        name="opening_right_frame",
    )

    tower.visual(
        Box((0.222, 0.105, 0.008)),
        origin=Origin(xyz=(0.0, 0.088, 0.209)),
        material=molded_polymer,
        name="throat_floor",
    )
    tower.visual(
        Box((0.222, 0.105, 0.008)),
        origin=Origin(xyz=(0.0, 0.088, 0.321)),
        material=molded_polymer,
        name="throat_top",
    )
    tower.visual(
        Box((0.008, 0.105, 0.112)),
        origin=Origin(xyz=(-0.111, 0.088, 0.265)),
        material=molded_polymer,
        name="throat_left",
    )
    tower.visual(
        Box((0.008, 0.105, 0.112)),
        origin=Origin(xyz=(0.111, 0.088, 0.265)),
        material=molded_polymer,
        name="throat_right",
    )

    tower.visual(
        Box((0.24, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.149, 0.319)),
        material=gasket_black,
        name="frame_gasket_top",
    )
    tower.visual(
        Box((0.24, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.149, 0.211)),
        material=gasket_black,
        name="frame_gasket_bottom",
    )
    tower.visual(
        Box((0.012, 0.006, 0.108)),
        origin=Origin(xyz=(-0.117, 0.149, 0.265)),
        material=gasket_black,
        name="frame_gasket_left",
    )
    tower.visual(
        Box((0.012, 0.006, 0.108)),
        origin=Origin(xyz=(0.117, 0.149, 0.265)),
        material=gasket_black,
        name="frame_gasket_right",
    )

    tower.visual(
        Box((0.20, 0.045, 0.008)),
        origin=Origin(xyz=(0.0, 0.116, 0.352)),
        material=darker_paint,
        name="hinge_mount_shelf",
    )
    tower.visual(
        Box((0.010, 0.034, 0.060)),
        origin=Origin(xyz=(-0.155, 0.121, 0.348)),
        material=darker_paint,
        name="left_hinge_bracket",
    )
    tower.visual(
        Box((0.010, 0.034, 0.060)),
        origin=Origin(xyz=(0.155, 0.121, 0.348)),
        material=darker_paint,
        name="right_hinge_bracket",
    )
    tower.visual(
        Box((0.036, 0.014, 0.012)),
        origin=Origin(xyz=(-0.150, 0.134, 0.382)),
        material=darker_paint,
        name="left_hinge_doubler",
    )
    tower.visual(
        Box((0.036, 0.014, 0.012)),
        origin=Origin(xyz=(0.150, 0.134, 0.382)),
        material=darker_paint,
        name="right_hinge_doubler",
    )
    tower.visual(
        Box((0.018, 0.018, 0.22)),
        origin=Origin(xyz=(-0.095, -0.145, 0.245)),
        material=darker_paint,
        name="rear_left_rib",
    )
    tower.visual(
        Box((0.018, 0.018, 0.22)),
        origin=Origin(xyz=(0.095, -0.145, 0.245)),
        material=darker_paint,
        name="rear_right_rib",
    )

    for i, x in enumerate((-0.18, 0.0, 0.18), start=1):
        add_bolt(tower, f"flashing_front_bolt_{i}", (x, 0.150, 0.010), radius=0.006)
        add_bolt(tower, f"flashing_back_bolt_{i}", (x, -0.150, 0.010), radius=0.006)
    for i, y in enumerate((-0.120, 0.120), start=1):
        add_bolt(tower, f"flashing_left_bolt_{i}", (-0.205, y, 0.010), radius=0.006)
        add_bolt(tower, f"flashing_right_bolt_{i}", (0.205, y, 0.010), radius=0.006)

    frame_bolts = [
        (-0.145, 0.154, 0.190),
        (0.145, 0.154, 0.190),
        (-0.145, 0.154, 0.340),
        (0.145, 0.154, 0.340),
        (-0.130, 0.154, 0.265),
        (0.130, 0.154, 0.265),
        (0.0, 0.154, 0.190),
        (0.0, 0.154, 0.340),
    ]
    for i, xyz in enumerate(frame_bolts, start=1):
        add_bolt(tower, f"frame_bolt_{i}", xyz, axis="y", radius=0.0045, length=0.014)

    hinge_bolts = [
        (-0.159, 0.123, 0.332),
        (-0.159, 0.123, 0.364),
        (0.159, 0.123, 0.332),
        (0.159, 0.123, 0.364),
        (-0.150, 0.134, 0.382),
        (0.150, 0.134, 0.382),
    ]
    for i, xyz in enumerate(hinge_bolts, start=1):
        add_bolt(tower, f"hinge_bolt_{i}", xyz, axis="x", radius=0.0045, length=0.014)

    tower.inertial = Inertial.from_geometry(
        Box((0.48, 0.42, 0.38)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.302, 0.170, 0.008)),
        origin=Origin(xyz=(0.0, 0.098, 0.010)),
        material=painted_steel,
        name="hood_top",
    )
    flap.visual(
        Box((0.302, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.178, -0.010)),
        material=painted_steel,
        name="front_lip",
    )
    flap.visual(
        Box((0.012, 0.155, 0.045)),
        origin=Origin(xyz=(-0.157, 0.095, -0.012)),
        material=painted_steel,
        name="left_skirt",
    )
    flap.visual(
        Box((0.012, 0.155, 0.045)),
        origin=Origin(xyz=(0.157, 0.095, -0.012)),
        material=painted_steel,
        name="right_skirt",
    )
    flap.visual(
        Box((0.302, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.024, -0.004)),
        material=darker_paint,
        name="rear_stiffener",
    )
    flap.visual(
        Box((0.025, 0.120, 0.020)),
        origin=Origin(xyz=(-0.080, 0.102, -0.004)),
        material=darker_paint,
        name="left_hood_rib",
    )
    flap.visual(
        Box((0.025, 0.120, 0.020)),
        origin=Origin(xyz=(0.080, 0.102, -0.004)),
        material=darker_paint,
        name="right_hood_rib",
    )
    flap.visual(
        Box((0.024, 0.016, 0.024)),
        origin=Origin(xyz=(-0.105, 0.006, 0.012)),
        material=darker_paint,
        name="left_hinge_leaf",
    )
    flap.visual(
        Box((0.024, 0.016, 0.024)),
        origin=Origin(xyz=(0.105, 0.006, 0.012)),
        material=darker_paint,
        name="right_hinge_leaf",
    )
    flap.visual(
        Box((0.016, 0.028, 0.016)),
        origin=Origin(xyz=(-0.105, 0.022, 0.016)),
        material=darker_paint,
        name="left_hinge_strap",
    )
    flap.visual(
        Box((0.016, 0.028, 0.016)),
        origin=Origin(xyz=(0.105, 0.022, 0.016)),
        material=darker_paint,
        name="right_hinge_strap",
    )
    flap.visual(
        Box((0.018, 0.012, 0.020)),
        origin=Origin(xyz=(-0.105, 0.004, 0.008)),
        material=darker_paint,
        name="left_pivot_mount",
    )
    flap.visual(
        Box((0.018, 0.012, 0.020)),
        origin=Origin(xyz=(0.105, 0.004, 0.008)),
        material=darker_paint,
        name="right_pivot_mount",
    )
    flap.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(
            xyz=(-0.105, 0.000, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=fastener_zinc,
        name="left_pivot_barrel",
    )
    flap.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(
            xyz=(0.105, 0.000, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=fastener_zinc,
        name="right_pivot_barrel",
    )

    flap_bolts = [
        (-0.110, 0.158, 0.014),
        (0.110, 0.158, 0.014),
        (-0.110, 0.060, 0.014),
        (0.110, 0.060, 0.014),
    ]
    for i, xyz in enumerate(flap_bolts, start=1):
        add_bolt(flap, f"hood_bolt_{i}", xyz, radius=0.0045, length=0.010)

    flap.inertial = Inertial.from_geometry(
        Box((0.31, 0.18, 0.06)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.085, -0.012)),
    )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent="tower",
        child="flap",
        origin=Origin(xyz=(0.0, 0.156, 0.388)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_aabb_contact("flap", "tower")
    ctx.expect_aabb_overlap("flap", "tower", axes="x", min_overlap=0.26)
    ctx.expect_origin_distance("flap", "tower", axes="x", max_dist=0.01)
    ctx.expect_aabb_gap(
        "flap",
        "tower",
        axis="y",
        max_gap=0.050,
        max_penetration=0.0,
        positive_elem="rear_stiffener",
        negative_elem="front_header",
        name="hinge_band_stays_close_to_front_header",
    )
    ctx.expect_aabb_gap(
        "flap",
        "tower",
        axis="y",
        max_gap=0.030,
        max_penetration=0.0,
        positive_elem="left_pivot_mount",
        negative_elem="left_hinge_bracket",
        name="left_pivot_mount_sits_near_left_hinge_bracket",
    )
    ctx.expect_aabb_gap(
        "flap",
        "tower",
        axis="y",
        max_gap=0.030,
        max_penetration=0.0,
        positive_elem="right_pivot_mount",
        negative_elem="right_hinge_bracket",
        name="right_pivot_mount_sits_near_right_hinge_bracket",
    )
    ctx.expect_aabb_gap(
        "flap",
        "tower",
        axis="z",
        max_gap=0.045,
        max_penetration=0.0,
        positive_elem="hood_top",
        negative_elem="opening_frame_header",
        name="closed_hood_stays_close_to_framed_outlet",
    )
    ctx.expect_joint_motion_axis(
        "flap_hinge",
        "flap",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(flap_hinge=1.0):
        ctx.expect_aabb_overlap("flap", "tower", axes="x", min_overlap=0.24)
        ctx.expect_aabb_gap(
            "flap",
            "tower",
            axis="z",
            min_gap=0.08,
            positive_elem="front_lip",
            negative_elem="opening_frame_header",
            name="open_flap_clears_the_outlet_header",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
